/* omxtx.c
 *
 * (c) 2012 Dickon Hood <dickon@fluff.org>
 *
 * A trivial OpenMAX transcoder for the Pi.
 *
 * Very much a work-in-progress, and as such is noisy, doesn't produce
 * particularly pretty output, and is probably buggier than a swamp in
 * summer.  Beware of memory leaks.
 *
 * Usage: ./omxtx input.foo output.m4v
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/* To do:
 *
 *  *  Flush the buffers at the end
 *  *  Sort out the PTSes
 *  *  Read up on buffer timings in general
 *  *  Feed the packets to AVFormat rather than dumping them raw to disc
 *  *  Interleave correctly with the other AV packets in the stream, rather
 *     than just dropping them entirely
 */

#define _BSD_SOURCE
#define FF_API_CODEC_ID

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "bcm_host.h"
#include "libavformat/avformat.h"
#include "libavutil/avutil.h"
#include "libavcodec/avcodec.h"
#include <error.h>

#include "OMX_Video.h"
#include "OMX_Types.h"
#include "OMX_Component.h"
#include "OMX_Core.h"
#include "OMX_Broadcom.h"

#include <pthread.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <time.h>
#include <errno.h>

#define SPECVERSIONMAJOR	(1)
#define SPECVERSIONMINOR	(1)
#define	SPECREVISION		(2)
#define SPECSTEP		(0)


static OMX_BUFFERHEADERTYPE *allocbufs(OMX_HANDLETYPE h, int port, int enable);

/* Hateful things: */
#define MAKEMEvar(y, x, l) do {	OMX_VERSIONTYPE *v;			\
				y = calloc(sizeof(x) + l, 1);		\
				y->nSize = sizeof(x) + l;		\
				v = (void *) &(((OMX_U32 *)y)[1]);	\
				v->s.nVersionMajor = SPECVERSIONMAJOR;	\
				v->s.nVersionMinor = SPECVERSIONMINOR;	\
				v->s.nRevision = SPECREVISION;		\
				v->s.nStep = SPECSTEP;			\
				} while (0) /* Yes, the void * is evil */

#define MAKEME(y, x)	 	MAKEMEvar(y, x, 0)

#define OERR(cmd)	do {						\
				/* usleep(10*1000); */			\
				oerr = cmd;				\
				if (oerr != OMX_ErrorNone) {		\
					fprintf(stderr, #cmd		\
						" failed on line %d: %x\n", __LINE__, oerr);	\
					exit(1);			\
				} else {				\
					fprintf(stderr, #cmd		\
						" completed OK.\n");	\
				}					\
			} while (0)

#define OERRq(cmd)	do {	oerr = cmd;				\
				if (oerr != OMX_ErrorNone) {		\
					fprintf(stderr, #cmd		\
						" failed: %x\n", oerr);	\
					exit(1);			\
				}					\
			} while (0)

/*
 * This has some sideeffects; don't remove calls to it without
 * understanding this.
 */
#define DUMPPORT(handle, port) do { OMX_VIDEO_PORTDEFINITIONTYPE *viddef; \
					portdef->nPortIndex = port;	\
					OERR(OMX_GetParameter(handle,	\
						OMX_IndexParamPortDefinition,\
						portdef));		\
	printf("Port %d is %s, %s\n", portdef->nPortIndex,		\
		(portdef->eDir == 0 ? "input" : "output"),		\
		(portdef->bEnabled == 0 ? "disabled" : "enabled"));	\
	printf("Wants %d bufs, needs %d, size %d, enabled: %d, pop: %d, " \
		"aligned %d\n", portdef->nBufferCountActual,		\
		portdef->nBufferCountMin, portdef->nBufferSize,		\
		portdef->bEnabled, portdef->bPopulated,			\
		portdef->nBufferAlignment);				\
	viddef = &portdef->format.video;				\
	if (portdef->eDomain == OMX_PortDomainVideo)			\
		printf("Video type is currently:\n"			\
			"\tMIME:\t\t%s\n"				\
			"\tNative:\t\t%p\n"				\
			"\tWidth:\t\t%d\n"				\
			"\tHeight:\t\t%d\n"				\
			"\tStride:\t\t%d\n"				\
			"\tSliceHeight:\t%d\n"				\
			"\tBitrate:\t%d\n"				\
			"\tFramerate:\t%d (%x); (%f)\n"			\
			"\tError hiding:\t%d\n"				\
			"\tCodec:\t\t%d\n"				\
			"\tColour:\t\t%d\n",				\
			viddef->cMIMEType, viddef->pNativeRender,	\
			viddef->nFrameWidth, viddef->nFrameHeight,	\
			viddef->nStride, viddef->nSliceHeight,		\
			viddef->nBitrate,				\
			viddef->xFramerate, viddef->xFramerate,		\
			((float)viddef->xFramerate/(float)65536),	\
			viddef->bFlagErrorConcealment,			\
			viddef->eCompressionFormat, viddef->eColorFormat); \
	} while (0);
/* ... but damn useful.*/



enum states {
	DECINIT,
	DECTUNNELSETUP,
	DECFLUSH,
	DECDONE,
	ENCPREINIT,
	ENCINIT,
	ENCGOTBUF,
	ENCDONE,
};



static struct context {
	AVFormatContext *ic;
	AVFormatContext *oc;
	int		nextin;
	int		nextout;
	int		incount;
	int		outcount;
	volatile int	fps;
	volatile int	framecount;
	int		done;
	volatile int	flags;
	OMX_BUFFERHEADERTYPE *encbufs, *bufhead;
	volatile enum states	decstate;
	volatile enum states	encstate;
	int		encportidx, decportidx;
	int		fd;
	OMX_HANDLETYPE	m2, m4;
	pthread_mutex_t	lock;
	AVPacket	*nextframe;
	AVPacket	*frameheads[1024];
} ctx;
#define FLAGS_VERBOSE		(1<<0)
#define FLAGS_DECEMPTIEDBUF	(1<<1)



static int mapcodec(enum CodecID id)
{
	printf("Mapping codec ID %d (%x)\n", id, id);
	switch (id) {
		case	CODEC_ID_MPEG2VIDEO:
		case	CODEC_ID_MPEG2VIDEO_XVMC:
			return OMX_VIDEO_CodingMPEG2;
		case	CODEC_ID_H264:
			return OMX_VIDEO_CodingAVC;
		default:
			return -1;
	}

	return -1;
}


static void dumpportstate(void)
{
	OMX_ERRORTYPE			oerr;
	OMX_PARAM_PORTDEFINITIONTYPE	*portdef;
	enum OMX_STATETYPE		state;

	printf("\n\nIn exit handler, after %d frames:\n", ctx.framecount);
	MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
	DUMPPORT(ctx.m2, ctx.decportidx);
	DUMPPORT(ctx.m2, ctx.decportidx+1);
	DUMPPORT(ctx.m4, ctx.encportidx);
	DUMPPORT(ctx.m4, ctx.encportidx+1);

	OMX_GetState(ctx.m2, &state);
	printf("Decoder state: %d\n", state);
	OMX_GetState(ctx.m4, &state);
	printf("Encoder state: %d\n", state);
}



OMX_ERRORTYPE deceventhandler(OMX_HANDLETYPE component,
				struct context *ctx,
				OMX_EVENTTYPE event, 
				OMX_U32 data1,
				OMX_U32 data2,
				OMX_PTR eventdata)
{
	OMX_ERRORTYPE	oerr;

	switch (event) {
	case OMX_EventError:
	if (ctx->flags & FLAGS_VERBOSE)
		printf("Decoder %p has errored: %x\n", component, data1);
		return data1;
		break;
	case OMX_EventCmdComplete:
	if (ctx->flags & FLAGS_VERBOSE)
		printf("Decoder %p has completed the last command.\n",
			component);
		break;
	case OMX_EventPortSettingsChanged: {
		OMX_PARAM_PORTDEFINITIONTYPE *portdef;
		MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
//	if (ctx->flags & FLAGS_VERBOSE)
		printf("Decoder %p port %d settings changed.\n", component,
			data1);
		DUMPPORT(component, data1);
		free(portdef);
		ctx->decstate = DECTUNNELSETUP;
	}
		break;
	default:
	if (ctx->flags & FLAGS_VERBOSE)
		printf("Got an event of type %x on decoder %p (d1: %x, d2 %x)\n", event, component, data1, data2);
	}
	return OMX_ErrorNone;
}



OMX_ERRORTYPE enceventhandler(OMX_HANDLETYPE component,
				struct context *ctx,
				OMX_EVENTTYPE event, 
				OMX_U32 data1,
				OMX_U32 data2,
				OMX_PTR eventdata)
{
	OMX_ERRORTYPE	oerr;

	switch (event) {
	case OMX_EventError:
	if (ctx->flags & FLAGS_VERBOSE)
		printf("Encoder %p has errored: %x\n", component, data1);
		return data1;
		break;
	case OMX_EventCmdComplete:
	if (ctx->flags & FLAGS_VERBOSE)
		printf("Encoder %p has completed the last command.\n",
			component);
		break;
	case OMX_EventPortSettingsChanged: {
		OMX_PARAM_PORTDEFINITIONTYPE *portdef;
		MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
//	if (ctx->flags & FLAGS_VERBOSE)
		printf("Encoder %p port %d settings changed.\n", component,
			data1);
		DUMPPORT(component, data1);
		free(portdef);
	}
		break;
	default:
	if (ctx->flags & FLAGS_VERBOSE)
		printf("Got an event of type %x on encoder %p (d1: %x, d2 %x)\n", event, component, data1, data2);
	}
	return OMX_ErrorNone;
}



OMX_ERRORTYPE emptied(OMX_HANDLETYPE component,
				struct context *ctx,
				OMX_BUFFERHEADERTYPE *buf)
{
	if (ctx->flags & FLAGS_VERBOSE)
		printf("Got a buffer emptied event on component %p, buf %p\n", component, buf);
	buf->nFilledLen = 0;
	ctx->flags |= FLAGS_DECEMPTIEDBUF;
	return OMX_ErrorNone;
}



OMX_ERRORTYPE filled(OMX_HANDLETYPE component,
				struct context *ctx,
				OMX_BUFFERHEADERTYPE *buf)
{
	OMX_BUFFERHEADERTYPE *spare;

	if (ctx->flags & FLAGS_VERBOSE)
		printf("Got buffer %p filled (len %d)\n", buf, buf->nFilledLen);

/*
 * Don't call OMX_FillThisBuffer() here, as the hardware craps out after
 * a short while.  I don't know why.  Reentrancy, or the like, I suspect.
 * Queue the packet(s) and deal with them in main().
 *
 * It only ever seems to ask for the one buffer, but better safe than sorry...
 */

	pthread_mutex_lock(&ctx->lock);
	if (ctx->bufhead == NULL) {
		buf->pAppPrivate = NULL;
		ctx->bufhead = buf;
		pthread_mutex_unlock(&ctx->lock);
		return OMX_ErrorNone;
	}

	spare = ctx->bufhead;
	while (spare->pAppPrivate != NULL)
		spare = spare->pAppPrivate;

	spare->pAppPrivate = buf;
	buf->pAppPrivate = NULL;
	pthread_mutex_unlock(&ctx->lock);

	return OMX_ErrorNone;
}


OMX_CALLBACKTYPE encevents = {
	(void (*)) enceventhandler,
	(void (*)) emptied,
	(void (*)) filled
};

OMX_CALLBACKTYPE decevents = {
	(void (*)) deceventhandler,
	(void (*)) emptied,
	(void (*)) filled
};



static void *fps(void *p)
{
	OMX_ERRORTYPE			oerr;
	OMX_PARAM_PORTDEFINITIONTYPE	*portdef;
	enum OMX_STATETYPE		state;
	int				lastframe;

	while (1) {
		lastframe = ctx.framecount;
		sleep(1);
		printf("Frame %6d (%5ds).  Frames last second: %d     \r",
			ctx.framecount, ctx.framecount/25,
				ctx.framecount-lastframe);
		fflush(stdout);
		if (0 && ctx.fps == 0) {
			printf("In fps thread, after %d frames:\n",
				ctx.framecount);
			MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
			DUMPPORT(ctx.m2, ctx.decportidx);
			DUMPPORT(ctx.m2, ctx.decportidx+1);
			DUMPPORT(ctx.m4, ctx.encportidx);
			DUMPPORT(ctx.m4, ctx.encportidx+1);

			OMX_GetState(ctx.m2, &state);
			printf("Decoder state: %d\n", state);
			OMX_GetState(ctx.m4, &state);
			printf("Encoder state: %d\n", state);
		}
	}
	return NULL;
}



static OMX_BUFFERHEADERTYPE *allocbufs(OMX_HANDLETYPE h, int port, int enable)
{
	int i;
	OMX_BUFFERHEADERTYPE *list = NULL, **end = &list;
	OMX_PARAM_PORTDEFINITIONTYPE *portdef;
	OMX_ERRORTYPE	oerr;

	MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
	portdef->nPortIndex = port;
	OERR(OMX_GetParameter(h, OMX_IndexParamPortDefinition, portdef));

	if (enable)
		OERR(OMX_SendCommand(h, OMX_CommandPortEnable, port, NULL));

	for (i = 0; i < portdef->nBufferCountActual; i++) {
		OMX_U8 *buf;

		buf = vcos_malloc_aligned(portdef->nBufferSize,
			portdef->nBufferAlignment, "buffer");
		printf("Allocated a buffer of %d bytes\n", portdef->nBufferSize);
		OERR(OMX_UseBuffer(h, end, port, NULL, portdef->nBufferSize,
			buf));
		end = (OMX_BUFFERHEADERTYPE **) &((*end)->pAppPrivate);
	}

	free(portdef);

	return list;
}



#define ENCNAME "OMX.broadcom.video_encode"
#define DECNAME "OMX.broadcom.video_decode"

int main(int argc, char *argv[])
{
	AVFormatContext	*ic;
	char		*iname;
	char		*oname;
	int		err;
	int		vidindex;
	int		i, j;
	OMX_ERRORTYPE	oerr;
	OMX_HANDLETYPE	m2 = NULL, m4 = NULL;
	OMX_VIDEO_PARAM_PORTFORMATTYPE	*pfmt;
	OMX_PORT_PARAM_TYPE		*porttype;
	OMX_PARAM_PORTDEFINITIONTYPE	*portdef;
	OMX_PARAM_U32TYPE		*omxi;
	OMX_PARAM_COMPONENTROLETYPE	*role;
	OMX_BUFFERHEADERTYPE		*encbufs, *decbufs;
	OMX_VIDEO_PORTDEFINITIONTYPE	*viddef;
	OMX_CONFIG_FRAMERATETYPE	*framerate;
	OMX_VIDEO_PARAM_PROFILELEVELTYPE *level;
	OMX_VIDEO_PARAM_BITRATETYPE	*bitrate;
	OMX_CONFIG_INTERLACETYPE	*interlace;
	OMX_PARAM_DATAUNITTYPE		*dataunit;
	OMX_NALSTREAMFORMATTYPE		*naltype;
	OMX_CONFIG_POINTTYPE		*pixaspect;
	AVBitStreamFilterContext	*bsfc;
	int		decportidx = 200;
	int		encportidx = 130;
	int		fd;
	pthread_t	fpst;
	pthread_attr_t	fpsa;
	time_t		start, end;
	int		offset;
	AVPacket	*p, *fp, *rp;
	int		ish264;
	int		filtertest;

	iname = argv[1];
	oname = argv[2];

	MAKEME(omxi, OMX_PARAM_U32TYPE);
	MAKEME(porttype, OMX_PORT_PARAM_TYPE);
	MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
	MAKEME(role, OMX_PARAM_COMPONENTROLETYPE);
	MAKEME(interlace, OMX_CONFIG_INTERLACETYPE);
	MAKEME(dataunit, OMX_PARAM_DATAUNITTYPE);
	MAKEME(naltype, OMX_NALSTREAMFORMATTYPE);
	MAKEME(pixaspect, OMX_CONFIG_POINTTYPE);

	av_register_all();

	ic = NULL;
	bsfc = NULL;
	ish264 = 0;
	pthread_mutex_init(&ctx.lock, NULL);

#if 0
	fmt = av_oformat_next(fmt);
	while (fmt) {
		printf("Found '%s'\t\t'%s'\n", fmt->name, fmt->long_name);
		fmt = av_oformat_next(fmt);
	}
#endif

	/* Input init: */

	if ((err = avformat_open_input(&ic, iname, NULL, NULL) != 0)) {
		fprintf(stderr, "Failed to open '%s': %s\n", iname,
			strerror(err));
		exit(1);
	}
	ctx.ic = ic;

	if (avformat_find_stream_info(ic, NULL) < 0) {
		fprintf(stderr, "Failed to find streams in '%s'\n", iname);
		exit(1);
	}

	av_dump_format(ic, 0, iname, 0);

	vidindex = av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO, -1, -1,
		NULL, 0);
	if (vidindex < 0) {
		fprintf(stderr, "Failed to find a video stream in '%s'\n",
			iname);
		exit(1);
	}
	printf("Found a video at index %d\n", vidindex);

	printf("Frame size: %dx%d\n", ic->streams[vidindex]->codec->width, 
		ic->streams[vidindex]->codec->height);
	ish264 = (ic->streams[vidindex]->codec->codec_id == CODEC_ID_H264);

	/* Output init: */
	ctx.fd = fd = open(oname, O_CREAT | O_LARGEFILE | O_WRONLY | O_TRUNC,
			0666);
	printf("File descriptor %d\n", fd);


#if 0
	avformat_alloc_output_context(&oc, NULL, /*NULL,*/ oname);
	if (!oc) {
		printf("Couldn't determine output from '%s'; using MPEG.\n",
			oname);
		avformat_alloc_output_context(&oc, NULL, /*"matroska",*/ oname);
	}
#endif
//	if (!oc)
//		exit(1);
	
//	fmt = oc->oformat;
	
	for (i = 0; i < ic->nb_streams; i++) {
		printf("Found stream %d, context %p\n",
			ic->streams[i]->index, ic->streams[i]->codec);
	}

	bcm_host_init();
	OERR(OMX_Init());
	OERR(OMX_GetHandle(&m2, DECNAME, &ctx, &decevents));
	OERR(OMX_GetHandle(&m4, ENCNAME, &ctx, &encevents));
	ctx.m2 = m2;
	ctx.m4 = m4;

	printf("Obtained handles.  %p decode, %p encode\n",
		m2, m4);

	OERR(OMX_GetParameter(m2, OMX_IndexParamVideoInit, porttype));
	printf("Found %d ports, starting at %d (%x) on decoder\n",
		porttype->nPorts, porttype->nStartPortNumber,
		porttype->nStartPortNumber);
	ctx.decportidx = decportidx = porttype->nStartPortNumber;

	OERR(OMX_GetParameter(m4, OMX_IndexParamVideoInit, porttype));
	printf("Found %d ports, starting at %d (%x) on encoder\n",
		porttype->nPorts, porttype->nStartPortNumber,
		porttype->nStartPortNumber);
	ctx.encportidx = encportidx = porttype->nStartPortNumber;

	OERR(OMX_SendCommand(m2, OMX_CommandPortDisable, decportidx, NULL));
	OERR(OMX_SendCommand(m2, OMX_CommandPortDisable, decportidx+1, NULL));
	OERR(OMX_SendCommand(m4, OMX_CommandPortDisable, encportidx, NULL));
	OERR(OMX_SendCommand(m4, OMX_CommandPortDisable, encportidx+1, NULL));

	DUMPPORT(m2, decportidx);
	viddef = &portdef->format.video;
	viddef->nFrameWidth = ic->streams[vidindex]->codec->width;
	viddef->nFrameHeight = ic->streams[vidindex]->codec->height;
	printf("Mapping codec %d to %d\n",
		ic->streams[vidindex]->codec->codec_id,
		mapcodec(ic->streams[vidindex]->codec->codec_id));
	viddef->eCompressionFormat = 
		mapcodec(ic->streams[vidindex]->codec->codec_id);
	viddef->bFlagErrorConcealment = 0;
//	viddef->xFramerate = 25<<16;
	OERR(OMX_SetParameter(m2, OMX_IndexParamPortDefinition, portdef));

#if 0
/* It appears these have limited effect: */
	dataunit->nPortIndex = decportidx;
	dataunit->eUnitType = OMX_DataUnitCodedPicture;
	dataunit->eEncapsulationType = OMX_DataEncapsulationGenericPayload;
	OERR(OMX_SetParameter(m2, OMX_IndexParamBrcmDataUnit, dataunit));

	if (ish264) {
		naltype->nPortIndex = decportidx;
		naltype->eNaluFormat = OMX_NaluFormatStartCodes;
		OERR(OMX_SetParameter(m2, OMX_IndexParamNalStreamFormatSelect,
			naltype));
	}
#endif

	DUMPPORT(m2, decportidx);
	DUMPPORT(m2, decportidx+1);
	DUMPPORT(m4, encportidx);
	DUMPPORT(m4, encportidx+1);

	OERR(OMX_SendCommand(m2, OMX_CommandStateSet, OMX_StateIdle, NULL));

	decbufs = allocbufs(m2, decportidx, 1);

/* Start the initial loop.  Process until we have a state change on port 131 */
	ctx.decstate = DECINIT;
	ctx.encstate = ENCPREINIT;
	OERR(OMX_SendCommand(m2, OMX_CommandStateSet, OMX_StateExecuting, NULL));

	rp = calloc(sizeof(AVPacket), 1);
	fp = calloc(sizeof(AVPacket), 1);
	filtertest = ish264;
	for (offset = i = j = 0; ctx.decstate != DECTUNNELSETUP && i < 120;
			i++, j++) {
		int rc;
		int k;
		int size, nsize;
		OMX_BUFFERHEADERTYPE *spare;

		if (offset == 0) {
			rc = av_read_frame(ic, rp);
			if (rc != 0) {
				perror("av_read_packet");
				break;
			}
			if (rp->stream_index != vidindex) {
				i--;
				av_free_packet(rp);
				continue;
			}
			size = rp->size;
			ctx.fps++;
			ctx.framecount++;

			if (ish264 && filtertest) {
				filtertest = 0;
				if (!(rp->data[0] == 0x00 &&
					rp->data[1] == 0x00 &&
					rp->data[2] == 0x00 &&
					rp->data[3] == 0x01)) {
					bsfc = av_bitstream_filter_init("h264_mp4toannexb");
					if (!bsfc) {
						printf("Failed to open filter."
							"  This is bad.\n");
					} else {
						printf("Have a filter at %p\n", bsfc);
					}
				}
			}
			if (bsfc) {
				p = fp;
				rc = av_bitstream_filter_filter(bsfc, ic->streams[vidindex]->codec,
						NULL, &(fp->data), &(fp->size),
						rp->data, rp->size,
						rp->flags & AV_PKT_FLAG_KEY);
				if (rc > 0) {
					av_free_packet(rp);
					p->destruct = av_destruct_packet;
				} else {
					printf("Failed to filter frame: "
						"%d (%x)\n", rc, rc);
					p = rp;
				}
			} else {
				p = rp;
			}
		}

initspareloop: /* Yeah, yeah... */
		ctx.flags &= ~FLAGS_DECEMPTIEDBUF;
		spare = decbufs;
		for (k = 0; spare && spare->nFilledLen != 0; k++)
			spare = spare->pAppPrivate;
		if (!spare) {
			usleep(10);
			goto initspareloop;	/* FIXME */
		}

//		printf("Using buffer %p (depth %d)\n", spare, k);
		if (size > spare->nAllocLen) {
			nsize = spare->nAllocLen;
		} else {
			nsize = size;
		}
		printf("Found a frame (%d), length %d (%d left, offset %d, nsize %d, buf %p, allocated %x, first bytes: 0x%02x%02x%02x%02x)\n", j, p->size, size, offset, nsize, spare, spare->nAllocLen, p->data[0], p->data[1], p->data[2], p->data[3]);

		memcpy(spare->pBuffer, &(p->data[offset]), nsize);
		spare->nFilledLen = nsize;
		spare->nOffset = 0;
		spare->nFlags = offset == 0 ? OMX_BUFFERFLAG_STARTTIME : 0;
		spare->nFlags |= size == nsize ? OMX_BUFFERFLAG_ENDOFFRAME : 0;
		OERR(OMX_EmptyThisBuffer(m2, spare));
		size -= nsize;
		if (size) {
			offset += nsize;
		} else {
			offset = 0;
			av_free_packet(p);
		}
	}

	if (ctx.decstate != DECTUNNELSETUP) {
		fprintf(stderr, "Failed to set the parameters after %d video"
			" frames.  Giving up.\n", i);
		DUMPPORT(m2, decportidx);
		DUMPPORT(m2, decportidx+1);
		DUMPPORT(m4, encportidx);
		DUMPPORT(m4, encportidx+1);
		exit(1);
	}

	printf("Decoder has changed settings.  Setting up encoder.\n");

	DUMPPORT(m2, decportidx+1);
	portdef->nPortIndex = encportidx;
	OERR(OMX_SetParameter(m4, OMX_IndexParamPortDefinition,
		portdef));
	OERR(OMX_SetupTunnel(m2, decportidx+1,
			m4, encportidx));
	OERR(OMX_SendCommand(m4, OMX_CommandStateSet, OMX_StateIdle, NULL));
	viddef = &portdef->format.video;
	if (viddef->nBitrate != 0) {
		viddef->nBitrate *= 3;
		viddef->nBitrate /= 4;
	} else {
		viddef->nBitrate = (1*1024*1024);
	}
//		viddef->nBitrate = (2*1024*1024);
//	viddef->nFrameWidth  /= 2;
//	viddef->nFrameHeight /= 2;

	viddef->eCompressionFormat = OMX_VIDEO_CodingAVC;
	viddef->nStride = viddef->nSliceHeight = viddef->eColorFormat = 0;
	portdef->nPortIndex = encportidx+1;
	OERR(OMX_SetParameter(m4, OMX_IndexParamPortDefinition, portdef));

	MAKEME(bitrate, OMX_VIDEO_PARAM_BITRATETYPE);
	bitrate->nPortIndex = encportidx+1;
	bitrate->eControlRate = OMX_Video_ControlRateVariable;
	bitrate->nTargetBitrate = viddef->nBitrate;
	OERR(OMX_SetParameter(m4, OMX_IndexParamVideoBitrate, bitrate));

	MAKEME(pfmt, OMX_VIDEO_PARAM_PORTFORMATTYPE);
	pfmt->nPortIndex = encportidx+1;
	pfmt->nIndex = 0;
	pfmt->eCompressionFormat = OMX_VIDEO_CodingAVC;
	pfmt->eColorFormat = 20;
	pfmt->xFramerate = viddef->xFramerate;

	pixaspect->nPortIndex = encportidx+1;
	pixaspect->nX = 64;
	pixaspect->nY = 45;
//	OERR(OMX_SetParameter(m2, OMX_IndexParamBrcmPixelAspectRatio, pixaspect));

	for (oerr = OMX_ErrorNone, i = 0; oerr == OMX_ErrorNone; i++) {
		pfmt->nIndex = i;
		oerr = OMX_GetParameter(m4, OMX_IndexParamVideoPortFormat,
			pfmt);
		if (oerr == OMX_ErrorNoMore)
			break;
		printf("Codecs supported:\n"
			"\tIndex:\t\t%d\n"
			"\tCodec:\t\t%d (%x)\n"
			"\tColour:\t\t%d\n"
			"\tFramerate:\t%x (%f)\n",
			pfmt->nIndex,
			pfmt->eCompressionFormat, pfmt->eCompressionFormat,
			pfmt->eColorFormat,
			pfmt->xFramerate,
			((float)pfmt->xFramerate/(float)65536));
	}
//		DUMPPORT(m4, encportidx+1); exit(0);

	pfmt->nPortIndex = encportidx+1;
	pfmt->nIndex = 1;
	pfmt->eCompressionFormat = OMX_VIDEO_CodingAVC;
	pfmt->eColorFormat = 0;
	pfmt->xFramerate = 0; //viddef->xFramerate;
	OERR(OMX_SetParameter(m4, OMX_IndexParamVideoPortFormat,
		pfmt));

	MAKEME(framerate, OMX_CONFIG_FRAMERATETYPE);
	framerate->nPortIndex = encportidx+1;
	framerate->xEncodeFramerate = viddef->xFramerate;
	OERR(OMX_SetParameter(m4, OMX_IndexConfigVideoFramerate, framerate));

#if 0 /* Doesn't seem to apply to video? */
printf("Interlacing: %d\n", ic->streams[vidindex]->codec->field_order);
	if (0 || ic->streams[vidindex]->codec->field_order == AV_FIELD_TT) {
		interlace->nPortIndex = encportidx+1;
		interlace->eMode = OMX_InterlaceFieldsInterleavedUpperFirst;
		interlace->bRepeatFirstField = 0;
		OERR(OMX_SetParameter(m4, OMX_IndexConfigCommonInterlace,
			interlace));
	}
#endif

	MAKEME(level, OMX_VIDEO_PARAM_PROFILELEVELTYPE);
	level->nPortIndex = encportidx+1;
	for (oerr = OMX_ErrorNone, i = 0; oerr == OMX_ErrorNone; i++) {
		level->nProfileIndex = i;
		oerr = OMX_GetParameter(m4,
			OMX_IndexParamVideoProfileLevelQuerySupported,
			level);
		if (oerr == OMX_ErrorNoMore)
			break;
		printf("Levels supported:\n"
			"\tIndex:\t\t%d\n"
			"\tProfile:\t%d\n"
			"\tLevel:\t\t%d\n",
			level->nProfileIndex,
			level->eProfile,
			level->eLevel);
	}
	OERR(OMX_GetParameter(m4, OMX_IndexParamVideoProfileLevelCurrent,
		level));
	printf("Current level:\t\t%d\nCurrent profile:\t%d\n",
		level->eLevel, level->eProfile);
	OERR(OMX_SetParameter(m4, OMX_IndexParamVideoProfileLevelCurrent,
		level));

	ctx.encbufs = encbufs = allocbufs(m4, encportidx+1, 1);
	DUMPPORT(m4, encportidx+1);
	OERR(OMX_SendCommand(m2, OMX_CommandPortEnable, decportidx+1, NULL));
	OERR(OMX_SendCommand(m4, OMX_CommandPortEnable, encportidx, NULL));
	OERR(OMX_SendCommand(m4, OMX_CommandStateSet,
		OMX_StateExecuting, NULL));
	sleep(1);
	OERR(OMX_FillThisBuffer(m4, encbufs));
DUMPPORT(m2, decportidx);
DUMPPORT(m2, decportidx+1);
DUMPPORT(m4, encportidx);
DUMPPORT(m4, encportidx+1);
printf("Starting main loop.\n");

	atexit(dumpportstate);
	pthread_attr_init(&fpsa);
	pthread_attr_setdetachstate(&fpsa, PTHREAD_CREATE_DETACHED);
	pthread_create(&fpst, &fpsa, fps, NULL);

	start = time(NULL);


/* Main loop: */
	for (offset = i = j = 0; ; i++, j++) {
		int rc;
		int k;
		size_t size, nsize;
		OMX_BUFFERHEADERTYPE *spare;

		if (offset == 0) {
			rc = av_read_frame(ic, rp);
			if (rc != 0) {
				perror("av_read_packet");
				break;
			}
			if (rp->stream_index != vidindex) {
				av_free_packet(p);
				i--;
				continue;
			}
			size = rp->size;
			ctx.fps++;
			ctx.framecount++;
			if (bsfc) {
				p = fp;
				rc = av_bitstream_filter_filter(bsfc, ic->streams[vidindex]->codec,
						NULL, &(fp->data), &(fp->size),
						rp->data, rp->size,
						rp->flags & AV_PKT_FLAG_KEY);
				if (rc > 0) {
					av_free_packet(rp);
					p->destruct = av_destruct_packet;
				} else {
					printf("Failed to filter frame: "
						"%d (%x)\n", rc, rc);
					p = rp;
				}
			} else {
				p = rp;
			}
		}

//		printf("Found a frame (%d), length %d\n", j, p->size);
spareloop: /* Yeah, yeah... */
		/* Do this frequently: */
	
		pthread_mutex_lock(&ctx.lock);
		spare = ctx.bufhead;
		ctx.bufhead = NULL;
		pthread_mutex_unlock(&ctx.lock);
		while (spare) {
			write(fd, &spare->pBuffer[spare->nOffset],
				spare->nFilledLen);
			spare->nFilledLen = 0;
			spare->nOffset = 0;
			OERRq(OMX_FillThisBuffer(m4, spare));
			spare = spare->pAppPrivate;
		}

		ctx.flags &= ~FLAGS_DECEMPTIEDBUF;
		spare = decbufs;
		for (k = 0; spare && spare->nFilledLen != 0; k++)
			spare = spare->pAppPrivate;
		if (!spare) {
//			printf("Error: failed to locate a spare buffer.  Frame %d\n", i);
/*
DUMPPORT(m2, decportidx);
DUMPPORT(m2, decportidx+1);
DUMPPORT(m4, encportidx);
DUMPPORT(m4, encportidx+1);
*/
//			while ((ctx.flags & FLAGS_DECEMPTIEDBUF) == 0);
			usleep(10);
			goto spareloop;	/* FIXME */
		}

//		printf("Using buffer %p (depth %d)\n", spare, k);
		if (size > spare->nAllocLen) {
			nsize = spare->nAllocLen;
		} else {
			nsize = size;
		}

		memcpy(spare->pBuffer, &p->data[offset], nsize);
		spare->nFilledLen = nsize;
		spare->nOffset = 0;
		spare->nFlags = OMX_BUFFERFLAG_TIME_UNKNOWN;
		OERRq(OMX_EmptyThisBuffer(m2, spare));
		size -= nsize;
		if (size) {
			offset += nsize;
		} else {
			offset = 0;
			av_free_packet(p);
		}
	}

	close(fd);

	end = time(NULL);

	printf("Processed %d frames in %d seconds; %df/s\n",
		ctx.framecount, end-start, (ctx.framecount/(end-start)));

exit(0);

	free(portdef);

	return 0;
}
