/*
Copyright 2014 Laxmikant Rashinkar
Copyright 2014-2017 Jay Sorg

Permission to use, copy, modify, distribute, and sell this software and its
documentation for any purpose is hereby granted without fee, provided that
the above copyright notice appear in all copies and that both that
copyright notice and this permission notice appear in supporting
documentation.

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
OPEN GROUP BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

capture

*/

#if defined(HAVE_CONFIG_H)
#include "config_ac.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

/* this should be before all X11 .h files */
#include <xorg-server.h>
#include <xorgVersion.h>

/* all driver need this */
#include <xf86.h>
#include <xf86_OSproc.h>

#include "rdp.h"
#include "rdpDraw.h"
#include "rdpClientCon.h"
#include "rdpReg.h"
#include "rdpMisc.h"
#include "rdpCapture.h"

#include "wyhash.h"
/* hex digits of pi as a 64 bit int */
#define WYHASH_SEED 0x3243f6a8885a308dull

#if defined(XORGXRDP_GLAMOR)
#include "rdpEgl.h"
#include <glamor.h>
#endif

#define LOG_LEVEL 1
#define LLOGLN(_level, _args) \
    do { if (_level < LOG_LEVEL) { ErrorF _args ; ErrorF("\n"); } } while (0)

/******************************************************************************/
/* copy rects with no error checking */
static int
rdpCopyBox_a8r8g8b8_to_a8r8g8b8(rdpClientCon *clientCon,
                                const uint8_t *src, int src_stride, int srcx, int srcy,
                                uint8_t *dst, int dst_stride, int dstx, int dsty,
                                BoxPtr rects, int num_rects)
{
    const uint8_t *s8;
    uint8_t *d8;
    int index;
    int jndex;
    int bytes;
    int height;
    BoxPtr box;

    for (index = 0; index < num_rects; index++)
    {
        box = rects + index;
        s8 = src + (box->y1 - srcy) * src_stride;
        s8 += (box->x1 - srcx) * 4;
        d8 = dst + (box->y1 - dsty) * dst_stride;
        d8 += (box->x1 - dstx) * 4;
        bytes = box->x2 - box->x1;
        bytes *= 4;
        height = box->y2 - box->y1;
        for (jndex = 0; jndex < height; jndex++)
        {
            g_memcpy(d8, s8, bytes);
            d8 += dst_stride;
            s8 += src_stride;
        }
    }
    return 0;
}

/******************************************************************************/
static int
rdpFillBox_yuvalp(int ax, int ay,
                  uint8_t *dst, int dst_stride)
{
    dst = dst + (ay << 8) * (dst_stride >> 8) + (ax << 8);
    g_memset(dst, 0, 64 * 64 * 4);
    return 0;
}

/******************************************************************************/
/* copy rects with no error checking
 * convert ARGB32 to 64x64 linear planar YUVA */
/* http://msdn.microsoft.com/en-us/library/ff635643.aspx
 * 0.299   -0.168935    0.499813
 * 0.587   -0.331665   -0.418531
 * 0.114    0.50059    -0.081282
   y = r *  0.299000 + g *  0.587000 + b *  0.114000;
   u = r * -0.168935 + g * -0.331665 + b *  0.500590;
   v = r *  0.499813 + g * -0.418531 + b * -0.081282; */
/* 19595  38470   7471
  -11071 -21736  32807
   32756 -27429  -5327 */
static int
rdpCopyBox_a8r8g8b8_to_yuvalp(int ax, int ay,
                              const uint8_t *src, int src_stride,
                              uint8_t *dst, int dst_stride,
                              BoxPtr rects, int num_rects)
{
    const uint8_t *s8;
    uint8_t *d8;
    uint8_t *yptr;
    uint8_t *uptr;
    uint8_t *vptr;
    uint8_t *aptr;
    const uint32_t *s32;
    int index;
    int jndex;
    int kndex;
    int width;
    int height;
    uint32_t pixel;
    uint8_t a;
    int r;
    int g;
    int b;
    int y;
    int u;
    int v;
    BoxPtr box;

    dst = dst + (ay << 8) * (dst_stride >> 8) + (ax << 8);
    for (index = 0; index < num_rects; index++)
    {
        box = rects + index;
        s8 = src + box->y1 * src_stride;
        s8 += box->x1 * 4;
        d8 = dst + (box->y1 - ay) * 64;
        d8 += box->x1 - ax;
        width = box->x2 - box->x1;
        height = box->y2 - box->y1;
        for (jndex = 0; jndex < height; jndex++)
        {
            s32 = (const uint32_t *) s8;
            yptr = d8;
            uptr = yptr + 64 * 64;
            vptr = uptr + 64 * 64;
            aptr = vptr + 64 * 64;
            kndex = 0;
            while (kndex < width)
            {
                pixel = *(s32++);
                a = (pixel >> 24) & UCHAR_MAX;
                r = (pixel >> 16) & UCHAR_MAX;
                g = (pixel >>  8) & UCHAR_MAX;
                b = (pixel >>  0) & UCHAR_MAX;
                y = (r *  19595 + g *  38470 + b *   7471) >> 16;
                u = (r * -11071 + g * -21736 + b *  32807) >> 16;
                v = (r *  32756 + g * -27429 + b *  -5327) >> 16;
                u = u + 128;
                v = v + 128;
                y = RDPCLAMP(y, 0, UCHAR_MAX);
                u = RDPCLAMP(u, 0, UCHAR_MAX);
                v = RDPCLAMP(v, 0, UCHAR_MAX);
                *(yptr++) = y;
                *(uptr++) = u;
                *(vptr++) = v;
                *(aptr++) = a;
                kndex++;
            }
            d8 += 64;
            s8 += src_stride;
        }
    }
    return 0;
}

/******************************************************************************/
int
a8r8g8b8_to_a8b8g8r8_box(const uint8_t *s8, int src_stride,
                         uint8_t *d8, int dst_stride,
                         int width, int height)
{
    int index;
    int jndex;
    int red;
    int green;
    int blue;
    const uint32_t *s32;
    uint32_t *d32;

    for (index = 0; index < height; index++)
    {
        s32 = (const uint32_t *) s8;
        d32 = (uint32_t *) d8;
        for (jndex = 0; jndex < width; jndex++)
        {
            SPLITCOLOR32(red, green, blue, *s32);
            *d32 = COLOR24(red, green, blue);
            s32++;
            d32++;
        }
        d8 += dst_stride;
        s8 += src_stride;
    }
    return 0;
}

/******************************************************************************/
/* copy rects with no error checking */
static int
rdpCopyBox_a8r8g8b8_to_a8b8g8r8(rdpClientCon *clientCon,
                                const uint8_t *src, int src_stride, int srcx, int srcy,
                                uint8_t *dst, int dst_stride, int dstx, int dsty,
                                BoxPtr rects, int num_rects)
{
    const uint8_t *s8;
    uint8_t *d8;
    int index;
    int width;
    int height;
    BoxPtr box;
    copy_box_proc copy_box;

    copy_box = clientCon->dev->a8r8g8b8_to_a8b8g8r8_box;
    for (index = 0; index < num_rects; index++)
    {
        box = rects + index;
        s8 = src + (box->y1 - srcy) * src_stride;
        s8 += (box->x1 - srcx) * 4;
        d8 = dst + (box->y1 - dsty) * dst_stride;
        d8 += (box->x1 - dstx) * 4;
        width = box->x2 - box->x1;
        height = box->y2 - box->y1;
        copy_box(s8, src_stride, d8, dst_stride, width, height);
    }
    return 0;
}

/******************************************************************************/
int
a8r8g8b8_to_r5g6b5_box(const uint8_t *s8, int src_stride,
                       uint8_t *d8, int dst_stride,
                       int width, int height)
{
    int index;
    int jndex;
    int red;
    int green;
    int blue;
    const uint32_t *s32;
    uint16_t *d16;

    for (index = 0; index < height; index++)
    {
        s32 = (const uint32_t *) s8;
        d16 = (uint16_t *) d8;
        for (jndex = 0; jndex < width; jndex++)
        {
            SPLITCOLOR32(red, green, blue, *s32);
            *d16 = COLOR16(red, green, blue);
            s32++;
            d16++;
        }
        d8 += dst_stride;
        s8 += src_stride;
    }
    return 0;
}

/******************************************************************************/
/* copy rects with no error checking */
static int
rdpCopyBox_a8r8g8b8_to_r5g6b5(rdpClientCon *clientCon,
                              const uint8_t *src, int src_stride, int srcx, int srcy,
                              uint8_t *dst, int dst_stride, int dstx, int dsty,
                              BoxPtr rects, int num_rects)
{
    const uint8_t *s8;
    uint8_t *d8;
    int index;
    int width;
    int height;
    BoxPtr box;
    copy_box_proc copy_box;

    copy_box = a8r8g8b8_to_r5g6b5_box; /* TODO, simd */
    for (index = 0; index < num_rects; index++)
    {
        box = rects + index;
        s8 = src + (box->y1 - srcy) * src_stride;
        s8 += (box->x1 - srcx) * 4;
        d8 = dst + (box->y1 - dsty) * dst_stride;
        d8 += (box->x1 - dstx) * 2;
        width = box->x2 - box->x1;
        height = box->y2 - box->y1;
        copy_box(s8, src_stride, d8, dst_stride, width, height);
    }
    return 0;
}

/******************************************************************************/
int
a8r8g8b8_to_a1r5g5b5_box(const uint8_t *s8, int src_stride,
                         uint8_t *d8, int dst_stride,
                         int width, int height)
{
    int index;
    int jndex;
    int red;
    int green;
    int blue;
    const uint32_t *s32;
    uint16_t *d16;

    for (index = 0; index < height; index++)
    {
        s32 = (const uint32_t *) s8;
        d16 = (uint16_t *) d8;
        for (jndex = 0; jndex < width; jndex++)
        {
            SPLITCOLOR32(red, green, blue, *s32);
            *d16 = COLOR15(red, green, blue);
            s32++;
            d16++;
        }
        d8 += dst_stride;
        s8 += src_stride;
    }
    return 0;
}

/******************************************************************************/
/* copy rects with no error checking */
static int
rdpCopyBox_a8r8g8b8_to_a1r5g5b5(rdpClientCon *clientCon,
                                const uint8_t *src, int src_stride, int srcx, int srcy,
                                uint8_t *dst, int dst_stride, int dstx, int dsty,
                                BoxPtr rects, int num_rects)
{
    const uint8_t *s8;
    uint8_t *d8;
    int index;
    int width;
    int height;
    BoxPtr box;
    copy_box_proc copy_box;

    copy_box = a8r8g8b8_to_a1r5g5b5_box; /* TODO, simd */
    for (index = 0; index < num_rects; index++)
    {
        box = rects + index;
        s8 = src + (box->y1 - srcy) * src_stride;
        s8 += (box->x1 - srcx) * 4;
        d8 = dst + (box->y1 - dsty) * dst_stride;
        d8 += (box->x1 - dstx) * 2;
        width = box->x2 - box->x1;
        height = box->y2 - box->y1;
        copy_box(s8, src_stride, d8, dst_stride, width, height);
    }
    return 0;
}

/******************************************************************************/
int
a8r8g8b8_to_r3g3b2_box(const uint8_t *s8, int src_stride,
                       uint8_t *d8, int dst_stride,
                       int width, int height)
{
    int index;
    int jndex;
    int red;
    int green;
    int blue;
    const uint32_t *s32;
    uint8_t *ld8;

    for (index = 0; index < height; index++)
    {
        s32 = (const uint32_t *) s8;
        ld8 = (uint8_t *) d8;
        for (jndex = 0; jndex < width; jndex++)
        {
            SPLITCOLOR32(red, green, blue, *s32);
            *ld8 = COLOR8(red, green, blue);
            s32++;
            ld8++;
        }
        d8 += dst_stride;
        s8 += src_stride;
    }
    return 0;
}

/******************************************************************************/
/* copy rects with no error checking */
static int
rdpCopyBox_a8r8g8b8_to_r3g3b2(rdpClientCon *clientCon,
                              const uint8_t *src, int src_stride, int srcx, int srcy,
                              uint8_t *dst, int dst_stride, int dstx, int dsty,
                              BoxPtr rects, int num_rects)
{
    const uint8_t *s8;
    uint8_t *d8;
    int index;
    int width;
    int height;
    BoxPtr box;
    copy_box_proc copy_box;

    copy_box = a8r8g8b8_to_r3g3b2_box; /* TODO, simd */
    for (index = 0; index < num_rects; index++)
    {
        box = rects + index;
        s8 = src + (box->y1 - srcy) * src_stride;
        s8 += (box->x1 - srcx) * 4;
        d8 = dst + (box->y1 - dsty) * dst_stride;
        d8 += (box->x1 - dstx) * 1;
        width = box->x2 - box->x1;
        height = box->y2 - box->y1;
        copy_box(s8, src_stride, d8, dst_stride, width, height);
    }
    return 0;
}

/******************************************************************************/
int
a8r8g8b8_to_nv12_box(const uint8_t *s8, int src_stride,
                     uint8_t *d8_y, int dst_stride_y,
                     uint8_t *d8_uv, int dst_stride_uv,
                     int width, int height)
{
    int index;
    int jndex;
    int R;
    int G;
    int B;
    int Y;
    int U;
    int V;
    int U_sum;
    int V_sum;
    int pixel;
    const uint32_t *s32a;
    const uint32_t *s32b;
    uint8_t *d8ya;
    uint8_t *d8yb;
    uint8_t *d8uv;

    for (jndex = 0; jndex < height; jndex += 2)
    {
        s32a = (const uint32_t *) (s8 + src_stride * jndex);
        s32b = (const uint32_t *) (s8 + src_stride * (jndex + 1));
        d8ya = d8_y + dst_stride_y * jndex;
        d8yb = d8_y + dst_stride_y * (jndex + 1);
        d8uv = d8_uv + dst_stride_uv * (jndex / 2);
        for (index = 0; index < width; index += 2)
        {
            U_sum = 0;
            V_sum = 0;

            pixel = s32a[0];
            s32a++;
            SPLITCOLOR32(R, G, B, pixel);
            Y = (( 66 * R + 129 * G +  25 * B + 128) >> 8) +  16;
            U = ((-38 * R -  74 * G + 112 * B + 128) >> 8) + 128;
            V = ((112 * R -  94 * G -  18 * B + 128) >> 8) + 128;
            d8ya[0] = RDPCLAMP(Y, 0, UCHAR_MAX);
            d8ya++;
            U_sum += RDPCLAMP(U, 0, UCHAR_MAX);
            V_sum += RDPCLAMP(V, 0, UCHAR_MAX);

            pixel = s32a[0];
            s32a++;
            SPLITCOLOR32(R, G, B, pixel);
            Y = (( 66 * R + 129 * G +  25 * B + 128) >> 8) +  16;
            U = ((-38 * R -  74 * G + 112 * B + 128) >> 8) + 128;
            V = ((112 * R -  94 * G -  18 * B + 128) >> 8) + 128;
            d8ya[0] = RDPCLAMP(Y, 0, UCHAR_MAX);
            d8ya++;
            U_sum += RDPCLAMP(U, 0, UCHAR_MAX);
            V_sum += RDPCLAMP(V, 0, UCHAR_MAX);

            pixel = s32b[0];
            s32b++;
            SPLITCOLOR32(R, G, B, pixel);
            Y = (( 66 * R + 129 * G +  25 * B + 128) >> 8) +  16;
            U = ((-38 * R -  74 * G + 112 * B + 128) >> 8) + 128;
            V = ((112 * R -  94 * G -  18 * B + 128) >> 8) + 128;
            d8yb[0] = RDPCLAMP(Y, 0, UCHAR_MAX);
            d8yb++;
            U_sum += RDPCLAMP(U, 0, UCHAR_MAX);
            V_sum += RDPCLAMP(V, 0, UCHAR_MAX);

            pixel = s32b[0];
            s32b++;
            SPLITCOLOR32(R, G, B, pixel);
            Y = (( 66 * R + 129 * G +  25 * B + 128) >> 8) +  16;
            U = ((-38 * R -  74 * G + 112 * B + 128) >> 8) + 128;
            V = ((112 * R -  94 * G -  18 * B + 128) >> 8) + 128;
            d8yb[0] = RDPCLAMP(Y, 0, UCHAR_MAX);
            d8yb++;
            U_sum += RDPCLAMP(U, 0, UCHAR_MAX);
            V_sum += RDPCLAMP(V, 0, UCHAR_MAX);

            d8uv[0] = (U_sum + 2) / 4;
            d8uv++;
            d8uv[0] = (V_sum + 2) / 4;
            d8uv++;
        }
    }
    return 0;
}

/******************************************************************************/
int
a8r8g8b8_to_nv12_709fr_box(const uint8_t *s8, int src_stride,
                           uint8_t *d8_y, int dst_stride_y,
                           uint8_t *d8_uv, int dst_stride_uv,
                           int width, int height)
{
    int index;
    int jndex;
    int R;
    int G;
    int B;
    int Y;
    int U;
    int V;
    int U_sum;
    int V_sum;
    int pixel;
    const uint32_t *s32a;
    const uint32_t *s32b;
    uint8_t *d8ya;
    uint8_t *d8yb;
    uint8_t *d8uv;

    for (jndex = 0; jndex < height; jndex += 2)
    {
        s32a = (const uint32_t *) (s8 + src_stride * jndex);
        s32b = (const uint32_t *) (s8 + src_stride * (jndex + 1));
        d8ya = d8_y + dst_stride_y * jndex;
        d8yb = d8_y + dst_stride_y * (jndex + 1);
        d8uv = d8_uv + dst_stride_uv * (jndex / 2);
        for (index = 0; index < width; index += 2)
        {
            U_sum = 0;
            V_sum = 0;

            pixel = s32a[0];
            s32a++;
            SPLITCOLOR32(R, G, B, pixel);
            Y =  ( 54 * R + 183 * G +  18 * B) >> 8;
            U = ((-29 * R -  99 * G + 128 * B) >> 8) + 128;
            V = ((128 * R - 116 * G -  12 * B) >> 8) + 128;
            d8ya[0] = RDPCLAMP(Y, 0, UCHAR_MAX);
            d8ya++;
            U_sum += RDPCLAMP(U, 0, UCHAR_MAX);
            V_sum += RDPCLAMP(V, 0, UCHAR_MAX);

            pixel = s32a[0];
            s32a++;
            SPLITCOLOR32(R, G, B, pixel);
            Y =  ( 54 * R + 183 * G +  18 * B) >> 8;
            U = ((-29 * R -  99 * G + 128 * B) >> 8) + 128;
            V = ((128 * R - 116 * G -  12 * B) >> 8) + 128;
            d8ya[0] = RDPCLAMP(Y, 0, UCHAR_MAX);
            d8ya++;
            U_sum += RDPCLAMP(U, 0, UCHAR_MAX);
            V_sum += RDPCLAMP(V, 0, UCHAR_MAX);

            pixel = s32b[0];
            s32b++;
            SPLITCOLOR32(R, G, B, pixel);
            Y =  ( 54 * R + 183 * G +  18 * B) >> 8;
            U = ((-29 * R -  99 * G + 128 * B) >> 8) + 128;
            V = ((128 * R - 116 * G -  12 * B) >> 8) + 128;
            d8yb[0] = RDPCLAMP(Y, 0, UCHAR_MAX);
            d8yb++;
            U_sum += RDPCLAMP(U, 0, UCHAR_MAX);
            V_sum += RDPCLAMP(V, 0, UCHAR_MAX);

            pixel = s32b[0];
            s32b++;
            SPLITCOLOR32(R, G, B, pixel);
            Y =  ( 54 * R + 183 * G +  18 * B) >> 8;
            U = ((-29 * R -  99 * G + 128 * B) >> 8) + 128;
            V = ((128 * R - 116 * G -  12 * B) >> 8) + 128;
            d8yb[0] = RDPCLAMP(Y, 0, UCHAR_MAX);
            d8yb++;
            U_sum += RDPCLAMP(U, 0, UCHAR_MAX);
            V_sum += RDPCLAMP(V, 0, UCHAR_MAX);

            d8uv[0] = (U_sum + 2) / 4;
            d8uv++;
            d8uv[0] = (V_sum + 2) / 4;
            d8uv++;
        }
    }
    return 0;
}

/******************************************************************************/
int
a8r8g8b8_to_yuv444_709fr_box(const uint8_t *s8, int src_stride,
                             uint8_t *d8, int dst_stride,
                             int width, int height)
{
    int index;
    int jndex;
    int R, G, B;
    int Y, U, V;
    int cY, cU, cV, cA;
    int pixel;
    const uint32_t *s32;
    uint32_t *d32;

    for (jndex = 0; jndex < height; jndex++)
    {
        s32 = (const uint32_t *) (s8 + src_stride * jndex);
        d32 = (uint32_t *) (d8 + dst_stride * jndex);
        for (index = 0; index < width; index++)
        {
            pixel = s32[0];
            s32++;
            SPLITCOLOR32(R, G, B, pixel);
            Y =  ( 54 * R + 183 * G +  18 * B) >> 8;
            U = ((-29 * R -  99 * G + 128 * B) >> 8) + 128;
            V = ((128 * R - 116 * G -  12 * B) >> 8) + 128;
            cY = RDPCLAMP(Y, 0, UCHAR_MAX) << 16;
            cU = RDPCLAMP(U, 0, UCHAR_MAX) << 8;
            cV = RDPCLAMP(V, 0, UCHAR_MAX);
            cA = 0xFF << 24;
            d32[0] = cY | cU | cV | cA;
            d32++;
        }
    }
    return 0;
}

/******************************************************************************/
/* copy rects with no error checking */
static int
rdpCopyBox_a8r8g8b8_to_nv12(rdpClientCon *clientCon,
                            const uint8_t *src, int src_stride,
                            int srcx, int srcy,
                            uint8_t *dst_y, int dst_stride_y,
                            uint8_t *dst_uv, int dst_stride_uv,
                            int dstx, int dsty,
                            BoxPtr rects, int num_rects)
{
    const uint8_t *s8;
    uint8_t *d8_y;
    uint8_t *d8_uv;
    int index;
    int width;
    int height;
    BoxPtr box;

    for (index = 0; index < num_rects; index++)
    {
        box = rects + index;
        s8 = src + (box->y1 - srcy) * src_stride;
        s8 += (box->x1 - srcx) * 4;
        d8_y = dst_y + (box->y1 - dsty) * dst_stride_y;
        d8_y += (box->x1 - dstx) * 1;
        d8_uv = dst_uv + ((box->y1 - dsty) / 2) * dst_stride_uv;
        d8_uv += (box->x1 - dstx) * 1;
        width = box->x2 - box->x1;
        height = box->y2 - box->y1;
        clientCon->dev->a8r8g8b8_to_nv12_box(s8, src_stride,
                                             d8_y, dst_stride_y,
                                             d8_uv, dst_stride_uv,
                                             width, height);
    }
    return 0;
}

/******************************************************************************/
static void
wyhash_rfx_tile_rows(const uint8_t *src, int src_stride, int x, int y,
    uint64_t *row_hashes, int nrows)
{
    int row;
    const uint8_t *s8;

    for(row = 0; row < nrows; row++)
    {
        s8 = src + (y+row) * src_stride + x * 4;
        row_hashes[row] = wyhash((const void*)s8, 64 * 4, WYHASH_SEED, _wyp);
    }
}

/******************************************************************************/
static uint64_t
wyhash_rfx_tile_from_rows(const uint64_t *tile_rows, int tile_row_stride, 
                          int x, int y)
{
    const uint64_t *row_hashes;
    row_hashes = tile_rows + (x / 64) * tile_row_stride + y;
    return wyhash((const void*)row_hashes, 64*sizeof(uint64_t), WYHASH_SEED, _wyp);
}

/* copy rects with no error checking */
static int
rdpCopyBox_a8r8g8b8_to_nv12_709fr(rdpClientCon *clientCon,
                                  const uint8_t *src, int src_stride,
                                  int srcx, int srcy,
                                  uint8_t *dst_y, int dst_stride_y,
                                  uint8_t *dst_uv, int dst_stride_uv,
                                  int dstx, int dsty,
                                  BoxPtr rects, int num_rects)
{
    const uint8_t *s8;
    uint8_t *d8_y;
    uint8_t *d8_uv;
    int index;
    int width;
    int height;
    BoxPtr box;

    for (index = 0; index < num_rects; index++)
    {
        box = rects + index;
        s8 = src + (box->y1 - srcy) * src_stride;
        s8 += (box->x1 - srcx) * 4;
        d8_y = dst_y + (box->y1 - dsty) * dst_stride_y;
        d8_y += (box->x1 - dstx) * 1;
        d8_uv = dst_uv + ((box->y1 - dsty) / 2) * dst_stride_uv;
        d8_uv += (box->x1 - dstx) * 1;
        width = box->x2 - box->x1;
        height = box->y2 - box->y1;
        clientCon->dev->a8r8g8b8_to_nv12_709fr_box(s8, src_stride,
                                                   d8_y, dst_stride_y,
                                                   d8_uv, dst_stride_uv,
                                                   width, height);
    }
    return 0;
}

/******************************************************************************/
#define MAX_LINEAR_COORDINATE(width, height, bytes_per_cell) \
    (((int)width) * ((int)height) * (bytes_per_cell))

/******************************************************************************/
#define XY_BYTE_COORDINATE(x, y, stride, bytes_per_cell) \
    ((((int)x) * bytes_per_cell) + (((int)y) * stride))

/******************************************************************************/
static uint8_t
RGB2Y(uint32_t R, uint32_t G, uint32_t B)
{
    const uint32_t Y = (54lu * R + 183lu * G + 18lu * B) >> 8lu;
    return (uint8_t)RDPCLAMP(Y, 0, UCHAR_MAX);
}

/******************************************************************************/
static uint8_t
RGB2U(uint32_t R, uint32_t G, uint32_t B)
{
    const uint32_t U = ((-29lu * R - 99lu * G + 128lu * B) >> 8lu) + 128lu;
    return (uint8_t)RDPCLAMP(U, 0, UCHAR_MAX);
}

/******************************************************************************/
static uint8_t
RGB2V(uint32_t R, uint32_t G, uint32_t B)
{
    const uint32_t V = ((128lu * R - 116lu * G - 12lu * B) >> 8lu) + 128lu;
    return (uint8_t)RDPCLAMP(V, 0, UCHAR_MAX);
}

/******************************************************************************/
static int
extractY(const uint8_t *image_data, int16_t x, int16_t y, 
         int16_t width, int16_t height, int image_stride, 
         int bytes_per_pixel, uint8_t *value) {
    const int offset = XY_BYTE_COORDINATE(x, y, image_stride, bytes_per_pixel);
    const int max_coord = MAX_LINEAR_COORDINATE(width, height, bytes_per_pixel);
    if (offset >= max_coord) {
        return 1;
    }
    const uint32_t pixel = *(const uint32_t*)(image_data + offset);
    uint32_t R, G, B;
    SPLITCOLOR32(R, G, B, pixel);
    *value = RGB2Y(R, G, B);
    return 0;
}

static int
extractU(const uint8_t *image_data, int16_t x, int16_t y, 
         int16_t width, int16_t height, int image_stride, 
         int bytes_per_pixel, uint8_t *value) {
    const int offset = XY_BYTE_COORDINATE(x, y, image_stride, bytes_per_pixel);
    const int max_coord = MAX_LINEAR_COORDINATE(width, height, bytes_per_pixel);
    if (offset >= max_coord) {
        return 1;
    }
    const uint32_t pixel = *(const uint32_t*)(image_data + offset);
    uint32_t R, G, B;
    SPLITCOLOR32(R, G, B, pixel);
    *value = RGB2U(R, G, B);
    return 0;
}

/******************************************************************************/
static int
extractV(const uint8_t *image_data, int16_t x, int16_t y, 
         int16_t width, int16_t height, int image_stride, 
         int bytes_per_pixel, uint8_t *value) {
    const int offset = XY_BYTE_COORDINATE(x, y, image_stride, bytes_per_pixel);
    const int max_coord = MAX_LINEAR_COORDINATE(width, height, bytes_per_pixel);
    if (offset >= max_coord) {
        return 1;
    }
    const uint32_t pixel = *(const uint32_t*)(image_data + offset);
    uint32_t R, G, B;
    SPLITCOLOR32(R, G, B, pixel);
    *value = RGB2V(R, G, B);
    return 0;
}

/******************************************************************************/
int
a8r8g8b8_to_yuv444_709fr_box_streamV2_cmp(const uint8_t *s8, int src_stride,
                                      uint8_t *dst_main_y, int dst_main_y_stride,
                                      uint8_t *dst_main_uv, int dst_main_uv_stride,
                                      uint8_t *dst_aux_y, int dst_aux_y_stride,
                                      uint8_t *dst_aux_uv, int dst_aux_uv_stride,
                                      const BoxPtr rect,
                                      int screen_width, int screen_height)
{
    const int src_bytes_per_pixel = 4;
    /*
        For unknown reasons, have to shrink the width/height by 4 to prevent
        segfaults.
    */
    int safety_width = screen_width - 4;
    int safety_height = screen_height - 4;

    uint16_t box_height = rect->y2 - rect->y1;
    uint16_t box_width = rect->x2 - rect->x1;

    uint16_t half_box_width = box_width / 2;
    uint16_t half_box_width_extent = min(rect->x1 + half_box_width, screen_width / 2);

    uint16_t half_box_height = box_height / 2;
    uint16_t half_box_height_extent = min(rect->y1 + half_box_height, screen_height / 2);

    uint16_t quarter_box_width = box_width / 4;
    uint16_t quarter_box_width_extent = min(rect->x1 + quarter_box_width, screen_width / 4);

    uint16_t half_screen_width = screen_width / 2;
    uint16_t quarter_screen_width = screen_width / 4;

    uint16_t x, y;
    uint8_t *yuv_dst_buffer;
    uint8_t extracted_value;

    for (y = rect->y1; y < rect->y2; ++y)
    {
        for (x = rect->x1; x < rect->x2; ++x)
        {
            // B1[x, y] = Y444[x, y]
            yuv_dst_buffer = dst_main_y + XY_BYTE_COORDINATE(x, y, dst_main_y_stride, 1);
            if (extractY(s8, x, y, safety_width, safety_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                continue;
            }
            yuv_dst_buffer[0] = extracted_value;
        }
    }

    for (y = max(0, ((int)rect->y1 - 1) / 2); y < half_box_height_extent; ++y)
    {
        for (x = max(0, ((int)rect->x1 - 1) / 2); x < half_box_width_extent; ++x)
        {
            uint8_t one, two, three, four;

            // B2[x, y] = U444[2 * x, 2 * y];
            yuv_dst_buffer = dst_main_uv + XY_BYTE_COORDINATE(x, y, dst_main_uv_stride, 2);
            if (!extractU(s8, 2 * x, 2 * y, safety_width, safety_height, src_stride, src_bytes_per_pixel, &one) &&
                !extractU(s8, 2 * x + 1, 2 * y, safety_width, safety_height, src_stride, src_bytes_per_pixel, &two) &&
                !extractU(s8, 2 * x, 2 * y + 1, safety_width, safety_height, src_stride, src_bytes_per_pixel, &three) &&
                !extractU(s8, 2 * x + 1, 2 * y + 1, safety_width, safety_height, src_stride, src_bytes_per_pixel, &four)
                )
            {
                yuv_dst_buffer[0] = (uint8_t)RDPCLAMP(((int)one + two + three + four + 2) / 4, 0, UCHAR_MAX);
            }

            // B3[x, y] = V444[2 * x, 2 * y];
            yuv_dst_buffer = dst_main_uv + XY_BYTE_COORDINATE(x, y, dst_main_uv_stride, 2);
            if (!extractV(s8, 2 * x, 2 * y, safety_width, safety_height, src_stride, src_bytes_per_pixel, &one) &&
                !extractV(s8, 2 * x + 1, 2 * y, safety_width, safety_height, src_stride, src_bytes_per_pixel, &two) &&
                !extractV(s8, 2 * x, 2 * y + 1, safety_width, safety_height, src_stride, src_bytes_per_pixel, &three) &&
                !extractV(s8, 2 * x + 1, 2 * y + 1, safety_width, safety_height, src_stride, src_bytes_per_pixel, &four)
                )
            {
                yuv_dst_buffer[1] = (uint8_t)RDPCLAMP(((int)one + two + three + four + 2) / 4, 0, UCHAR_MAX);
            }
        }
    }

    ////////////////////////// END MAIN VIEW ///////////////////////////////////

    for (y = rect->y1; y < rect->y2; ++y)
    {
        for (x = max(0, ((int)rect->x1 - 1) / 2); x < half_box_width_extent; ++x)
        {
            // B4[x, y] = U444[2x + 1, y];
            yuv_dst_buffer = dst_aux_y + XY_BYTE_COORDINATE(x, y, dst_aux_y_stride, 1);
            if (!extractU(s8, 2 * x + 1, y, safety_width, safety_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                yuv_dst_buffer[0] = extracted_value;
            }

            // B5[(W/2) + x, y] = V444[2x + 1, y];
            yuv_dst_buffer = dst_aux_y + XY_BYTE_COORDINATE(x + half_screen_width, y, dst_aux_y_stride, 1);
            if (!extractV(s8, 2 * x + 1, y, safety_width, safety_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                yuv_dst_buffer[0] = extracted_value;
            }
        }
    }

    for (y = max(0, ((int)rect->y1 - 1) / 2); y < half_box_height_extent; ++y)
    {
        for (x = max(0, rect->x1 / 4); x < quarter_box_width_extent; ++x)
        {
            // B6[x, y] = U444[4 * x, 2 * y + 1];
            yuv_dst_buffer = dst_aux_uv + XY_BYTE_COORDINATE(x, y, dst_aux_uv_stride, 2);
            if (!extractU(s8, 4 * x, 2 * y + 1, safety_width, safety_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                yuv_dst_buffer[0] = extracted_value;
            }
            // B7[(W/4) + x, y] = V444[4 * x, 2 * y + 1];
            yuv_dst_buffer = dst_aux_uv + XY_BYTE_COORDINATE(x + quarter_screen_width, y, dst_aux_uv_stride, 2);
            if (!extractV(s8, 4 * x, 2 * y + 1, safety_width, safety_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                yuv_dst_buffer[0] = extracted_value;
            }
        }
    }

    for (y = max(0, ((int)rect->y1 - 1) / 2); y < half_box_height_extent; ++y)
    {
        for (x = max(0, ((int)rect->x1 - 2) / 4); x < quarter_box_width_extent; ++x)
        {
            // B8[x, y] = U444[4 * x + 2, 2 * y + 1];
            yuv_dst_buffer = dst_aux_uv + XY_BYTE_COORDINATE(x, y, dst_aux_uv_stride, 2);
            if (!extractU(s8, 4 * x + 2, 2 * y + 1, safety_width, safety_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                yuv_dst_buffer[1] = extracted_value;
            }
            // B9[(W/4) + x, y] = V444[4 * x + 2, 2 * y + 1];
            yuv_dst_buffer = dst_aux_uv + XY_BYTE_COORDINATE(x + quarter_screen_width, y, dst_aux_uv_stride, 2);
            if (!extractV(s8, 4 * x + 2, 2 * y + 1, safety_width, safety_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                yuv_dst_buffer[1] = extracted_value;
            }
        }
    }
    return 0;
}

/******************************************************************************/
static void general_RGBToAVC444YUVv2_ANY_DOUBLE_ROW(
    const uint8_t* srcEven, const uint8_t* srcOdd,
    uint8_t* yLumaDstEven, uint8_t* yLumaDstOdd, uint8_t* uvLumaDst,
    uint8_t* yEvenChromaDst1, uint8_t* yEvenChromaDst2,
    uint8_t* yOddChromaDst1, uint8_t* yOddChromaDst2,
    uint8_t* uChromaDst1, uint8_t* uChromaDst2,
    uint8_t* vChromaDst1, uint8_t* vChromaDst2, const BoxPtr rect)
{
    uint16_t x;
    const uint32_t bpp = 4;
    uint32_t b, g, r;
    for (x = rect->x1; x < rect->x2; x += 2)
    {
        uint8_t Ya, Ua, Va;
        uint8_t Yb, Ub, Vb;
        uint8_t Yc, Uc, Vc;
        uint8_t Yd, Ud, Vd;
        {
            srcEven += bpp;
            SPLITCOLOR32(r, g, b, *((uint32_t*)srcEven));
            Ya = RGB2Y(r, g, b);
            Ua = RGB2U(r, g, b);
            Va = RGB2V(r, g, b);
        }

        if (x < rect->x2 - 1)
        {
            srcEven += bpp;
            SPLITCOLOR32(r, g, b, *((uint32_t*)srcEven));
            Yb = RGB2Y(r, g, b);
            Ub = RGB2U(r, g, b);
            Vb = RGB2V(r, g, b);
        }
        else
        {
            Yb = Ya;
            Ub = Ua;
            Vb = Va;
        }

        if (srcOdd)
        {
            srcOdd += bpp;
            SPLITCOLOR32(r, g, b, *((uint32_t*)srcOdd));
            Yc = RGB2Y(r, g, b);
            Uc = RGB2U(r, g, b);
            Vc = RGB2V(r, g, b);
        }
        else
        {
            Yc = Ya;
            Uc = Ua;
            Vc = Va;
        }

        if (srcOdd && (x < rect->x2 - 1))
        {
            srcOdd += bpp;
            SPLITCOLOR32(r, g, b, *((uint32_t*)srcOdd));
            Yd = RGB2Y(r, g, b);
            Ud = RGB2U(r, g, b);
            Vd = RGB2V(r, g, b);
        }
        else
        {
            Yd = Ya;
            Ud = Ua;
            Vd = Va;
        }

        /* Y [b1] */
        *yLumaDstEven++ = Ya;

        if (x < rect->x2 - 1)
        {
            *yLumaDstEven++ = Yb;
        }

        if (srcOdd)
        {
            *yLumaDstOdd++ = Yc;
        }

        if (srcOdd && (x < rect->x2 - 1))
        {
            *yLumaDstOdd++ = Yd;
        }

        /* 2x 2y [b2,b3] */
        *uvLumaDst++ = (Ua + Ub + Uc + Ud) / 4;
        *uvLumaDst++ = (Va + Vb + Vc + Vd) / 4;

        /* 2x+1, y [b4,b5] even */
        if (x < rect->x2 - 1)
        {
            *yEvenChromaDst1++ = Ub;
            *yEvenChromaDst2++ = Vb;
        }

        if (srcOdd)
        {
            /* 2x+1, y [b4,b5] odd */
            if (x < rect->x2 - 1)
            {
                *yOddChromaDst1++ = Ud;
                *yOddChromaDst2++ = Vd;
            }

            /* 4x 2y+1 [b6, b7] */
            if (x % 4 == 0)
            {
                *uChromaDst1 = Uc;
                uChromaDst1 += 2;

                *uChromaDst2 = Vc;
                uChromaDst2 += 2;
            }
            /* 4x+2 2y+1 [b8, b9] */
            else
            {
                *vChromaDst1 = Uc;
                vChromaDst1 += 2;

                *vChromaDst2 = Vc;
                vChromaDst2 += 2;
            }
        }
    }
}

/******************************************************************************/
static int general_RGBToAVC444YUVv2_ANY(const uint8_t* pSrc,
                                        uint32_t srcStep, 
                                        uint8_t* pDst1[3],
                                        const uint32_t dst1Step[3], 
                                        uint8_t* pDst2[3],
                                        const uint32_t dst2Step[3],
                                        const BoxPtr rect,
                                        const uint32_t screen_width)
{
    if ((rect->y2 - rect->y1) < 1 || (rect->x2 - rect->x1) < 1)
        return 1;

    const int16_t half_screen_width = screen_width / 2;
    const uint16_t x = rect->x1;
    const uint16_t half_x = x / 2;
    uint16_t y;
    for (y = rect->y1; y < rect->y2; y += 2)
    {
        uint16_t half_y = y / 2;
        const uint8_t* srcEven = (pSrc + y * srcStep) + (x * 4);
        const uint8_t* srcOdd = (y < rect->y2 - 1) ? (srcEven + srcStep) : NULL;
        uint8_t* dstLumaYEven = (pDst1[0] + y * dst1Step[0]) + x;
        uint8_t* dstLumaYOdd = (dstLumaYEven + dst1Step[0]);
        uint8_t* dstLumaUV = (pDst1[1] + half_y * dst1Step[1]) + x;
        uint8_t* dstEvenChromaY1 = (pDst2[0] + y * dst2Step[0]) + half_x;
        uint8_t* dstEvenChromaY2 = dstEvenChromaY1 + half_screen_width;
        uint8_t* dstOddChromaY1 = dstEvenChromaY1 + dst2Step[0];
        uint8_t* dstOddChromaY2 = dstEvenChromaY2 + dst2Step[0];
        uint8_t* dstChromaU1 = (pDst2[1] + half_y * dst2Step[1]) + half_x;
        uint8_t* dstChromaV1 = dstChromaU1 + 1;
        uint8_t* dstChromaU2 = dstChromaU1 + half_screen_width;
        uint8_t* dstChromaV2 = dstChromaV1 + half_screen_width;
        general_RGBToAVC444YUVv2_ANY_DOUBLE_ROW(
            srcEven, srcOdd, dstLumaYEven, dstLumaYOdd, dstLumaUV,
            dstEvenChromaY1, dstEvenChromaY2, dstOddChromaY1, dstOddChromaY2,
            dstChromaU1, dstChromaU2, dstChromaV1, dstChromaV2, rect);
    }

    return 0;
}

/******************************************************************************/
int
a8r8g8b8_to_yuv444_709fr_box_streamV2_freerdp(const uint8_t *s8, int src_stride,
                                      uint8_t *dst_main_y, int dst_main_y_stride,
                                      uint8_t *dst_main_uv, int dst_main_uv_stride,
                                      uint8_t *dst_aux_y, int dst_aux_y_stride,
                                      uint8_t *dst_aux_uv, int dst_aux_uv_stride,
                                      const BoxPtr rect,
                                      int screen_width)
{
    uint8_t* pDst1[2] = {dst_main_y, dst_main_uv};
    uint32_t pStep1[2] = {dst_main_y_stride, dst_main_uv_stride};

    uint8_t* pDst2[2] = {dst_aux_y, dst_aux_uv};
    uint32_t pStep2[2] = {dst_aux_y_stride, dst_aux_uv_stride};

    general_RGBToAVC444YUVv2_ANY(
        s8,
        src_stride,
        pDst1, pStep1,
        pDst2, pStep2,
        rect,
        screen_width
    );
    return 0;
}

struct xrdp_rectangle
{
    int16_t	x, y, width, height;
};

/******************************************************************************/
static void
snapRectangleToGrid(struct xrdp_rectangle *rect_buffer)
{
    LLOGLN(10, ("snapRectangleToGrid:"));
    // Round x and y down to nearest multiple of 16
    rect_buffer->x = (rect_buffer->x / 16) * 16;
    rect_buffer->y = (rect_buffer->y / 16) * 16;
    // Round width and height up to nearest multiple of 16
    rect_buffer->width = ((rect_buffer->width + 15) / 16) * 16;
    rect_buffer->height = ((rect_buffer->height + 15) / 16) * 16;
    // Calculate new x and y coordinates
    int x_diff = rect_buffer->x % 16;
    int y_diff = rect_buffer->y % 16;
    rect_buffer->x -= x_diff;
    rect_buffer->y -= y_diff;
    // Add any difference to width and height
    rect_buffer->width += x_diff;
    rect_buffer->height += y_diff;
}

/******************************************************************************/
static int
rdpCopyBox_yuv444_to_streamV2(rdpClientCon *clientCon,
                              const uint8_t *src, int src_stride,
                              uint8_t *dst_main_y, int dst_main_y_stride,
                              uint8_t *dst_main_uv, int dst_main_uv_stride,
                              uint8_t *dst_aux_y, int dst_aux_y_stride,
                              uint8_t *dst_aux_uv, int dst_aux_uv_stride,
                              int srcx, int srcy,
                              int dstx, int dsty,
                              const BoxPtr rects, int num_rects,
                              int screen_width, int screen_height)

{
    const uint8_t *s8;
    uint8_t *d8_main_y;
    uint8_t *d8_main_uv;
    uint8_t *d8_aux_y;
    uint8_t *d8_aux_uv;
    int index;

    s8 = src;
    d8_main_y = dst_main_y;
    d8_main_uv = dst_main_uv;

    d8_aux_y = dst_aux_y;
    d8_aux_uv = dst_aux_uv;

    for (index = 0; index < num_rects; ++index)
    {
        const BoxPtr rect = rects + index;
        a8r8g8b8_to_yuv444_709fr_box_streamV2_freerdp(s8, src_stride,
                                              d8_main_y, dst_main_y_stride,
                                              d8_main_uv, dst_main_uv_stride,
                                              d8_aux_y, dst_aux_y_stride,
                                              d8_aux_uv, dst_aux_uv_stride,
                                              rect,
                                              screen_width);
        // a8r8g8b8_to_yuv444_709fr_box_streamV2_cmp(s8, src_stride,
        //                                       d8_main_y, dst_main_y_stride,
        //                                       d8_main_uv, dst_main_uv_stride,
        //                                       d8_aux_y, dst_aux_y_stride,
        //                                       d8_aux_uv, dst_aux_uv_stride,
        //                                       rect, screen_width, screen_height);
    }
    return 0;
}

/******************************************************************************/
static Bool
rdpCopyBoxList(rdpClientCon *clientCon, PixmapPtr dstPixmap,
               BoxPtr out_rects, int num_out_rects)
{
    PixmapPtr hwPixmap;
    BoxPtr pbox;
    ScreenPtr pScreen;
    GCPtr copyGC;
    ChangeGCVal tmpval[1];
    int count;
    int index;
    int left;
    int top;
    int width;
    int height;
    char pix1[16];
    rdpPtr dev;

    LLOGLN(10, ("rdpCopyBoxList:"));

    dev = clientCon->dev;
    pScreen = dev->pScreen;
    hwPixmap = pScreen->GetScreenPixmap(pScreen);
    copyGC = GetScratchGC(dev->depth, pScreen);
    if (copyGC == NULL)
    {
        return FALSE;
    }
    tmpval[0].val = GXcopy;
    ChangeGC(NullClient, copyGC, GCFunction, tmpval);
    ValidateGC(&(hwPixmap->drawable), copyGC);
    count = num_out_rects;
    pbox = out_rects;
    for (index = 0; index < count; index++)
    {
        left = pbox[index].x1;
        top = pbox[index].y1;
        width = pbox[index].x2 - pbox[index].x1;
        height = pbox[index].y2 - pbox[index].y1;
        if ((width > 0) && (height > 0))
        {
            copyGC->ops->CopyArea(&(hwPixmap->drawable),
                                    &(dstPixmap->drawable),
                                    copyGC, left, top,
                                    width, height, left, top);
        }
    }
    FreeScratchGC(copyGC);
    pScreen->GetImage(&(dstPixmap->drawable), 0, 0, 1, 1, ZPixmap,
                          0xffffffff, pix1);

    return TRUE;
}

/******************************************************************************/
static Bool
isShmStatusActive(enum shared_memory_status status) {
    switch (status) {
        case SHM_ACTIVE:
        case SHM_RFX_ACTIVE:
        case SHM_H264_ACTIVE:
            return TRUE;
        default:
            return FALSE;
    }
}

/******************************************************************************/
static Bool
rdpCapture0(rdpClientCon *clientCon, RegionPtr in_reg, BoxPtr *out_rects,
            int *num_out_rects, struct image_data *id)
{
    BoxPtr psrc_rects;
    BoxRec rect;
    int num_rects;
    int i;
    Bool rv;
    const uint8_t *src;
    uint8_t *dst;
    int src_stride;
    int dst_stride;
    int dst_format;

    LLOGLN(10, ("rdpCapture0:"));

    if (!isShmStatusActive(clientCon->shmemstatus)) {
        LLOGLN(0, ("rdpCapture0: WARNING -- Shared memory is not configured. Aborting capture!"));
        return FALSE;
    }

    rv = TRUE;

    num_rects = REGION_NUM_RECTS(in_reg);
    psrc_rects = REGION_RECTS(in_reg);

    if (num_rects < 1)
    {
        return FALSE;
    }

    *num_out_rects = num_rects;

    *out_rects = g_new(BoxRec, num_rects);
    for (i = 0; i < num_rects; i++)
    {
        rect = psrc_rects[i];
        (*out_rects)[i] = rect;
    }

    if (clientCon->dev->glamor || clientCon->dev->nvidia)
    {
        /* copy vmem to smem */
        if (!rdpCopyBoxList(clientCon, clientCon->dev->screenSwPixmap,
                            *out_rects, *num_out_rects))
        {
            return FALSE;
        }
    }

    src = id->pixels;
    dst = id->shmem_pixels;
    dst_format = clientCon->rdp_format;
    src_stride = id->lineBytes;
    dst_stride = clientCon->cap_stride_bytes;

    if (dst_format == XRDP_a8r8g8b8)
    {
        rdpCopyBox_a8r8g8b8_to_a8r8g8b8(clientCon,
                                        src, src_stride, 0, 0,
                                        dst, dst_stride, 0, 0,
                                        psrc_rects, num_rects);
    }
    else if (dst_format == XRDP_a8b8g8r8)
    {
        rdpCopyBox_a8r8g8b8_to_a8b8g8r8(clientCon,
                                        src, src_stride, 0, 0,
                                        dst, dst_stride, 0, 0,
                                        psrc_rects, num_rects);
    }
    else if (dst_format == XRDP_r5g6b5)
    {
        rdpCopyBox_a8r8g8b8_to_r5g6b5(clientCon,
                                      src, src_stride, 0, 0,
                                      dst, dst_stride, 0, 0,
                                      psrc_rects, num_rects);
    }
    else if (dst_format == XRDP_a1r5g5b5)
    {
        rdpCopyBox_a8r8g8b8_to_a1r5g5b5(clientCon,
                                        src, src_stride, 0, 0,
                                        dst, dst_stride, 0, 0,
                                        psrc_rects, num_rects);
    }
    else if (dst_format == XRDP_r3g3b2)
    {
        rdpCopyBox_a8r8g8b8_to_r3g3b2(clientCon,
                                      src, src_stride, 0, 0,
                                      dst, dst_stride, 0, 0,
                                      psrc_rects, num_rects);
    }
    else
    {
        LLOGLN(0, ("rdpCapture0: unimplemented color conversion"));
    }
    return rv;
}

/******************************************************************************/
/* make out_rects always multiple of 16 width and height */
static Bool
rdpCapture1(rdpClientCon *clientCon, RegionPtr in_reg, BoxPtr *out_rects,
            int *num_out_rects, struct image_data *id)
{
    BoxPtr psrc_rects;
    BoxRec rect;
    BoxRec srect;
    const uint8_t *src_rect;
    uint8_t *dst_rect;
    int num_rects;
    int src_bytespp;
    int dst_bytespp;
    int width;
    int height;
    int src_offset;
    int dst_offset;
    int index;
    int jndex;
    int kndex;
    int red;
    int green;
    int blue;
    int ex;
    int ey;
    Bool rv;
    const uint32_t *s32;
    uint32_t *d32;
    const uint8_t *src;
    uint8_t *dst;
    int src_stride;
    int dst_stride;
    int dst_format;

    LLOGLN(10, ("rdpCapture1:"));

    if (!isShmStatusActive(clientCon->shmemstatus)) {
        LLOGLN(0, ("rdpCapture1: WARNING -- Shared memory is not configured. Aborting capture!"));
        return FALSE;
    }

    rv = TRUE;

    num_rects = REGION_NUM_RECTS(in_reg);
    psrc_rects = REGION_RECTS(in_reg);

    if (num_rects < 1)
    {
        return FALSE;
    }

    srect.x1 = clientCon->cap_left;
    srect.y1 = clientCon->cap_top;
    srect.x2 = clientCon->cap_left + clientCon->cap_width;
    srect.y2 = clientCon->cap_top + clientCon->cap_height;

    *num_out_rects = num_rects;

    *out_rects = g_new(BoxRec, num_rects * 4);
    index = 0;
    while (index < num_rects)
    {
        rect = psrc_rects[index];
        width = rect.x2 - rect.x1;
        height = rect.y2 - rect.y1;
        ex = ((width + 15) & ~15) - width;
        if (ex != 0)
        {
            rect.x2 += ex;
            if (rect.x2 > srect.x2)
            {
                rect.x1 -= rect.x2 - srect.x2;
                rect.x2 = srect.x2;
            }
            if (rect.x1 < srect.x1)
            {
                rect.x1 += 16;
            }
        }
        ey = ((height + 15) & ~15) - height;
        if (ey != 0)
        {
            rect.y2 += ey;
            if (rect.y2 > srect.y2)
            {
                rect.y1 -= rect.y2 - srect.y2;
                rect.y2 = srect.y2;
            }
            if (rect.y1 < srect.y1)
            {
                rect.y1 += 16;
            }
        }
        (*out_rects)[index] = rect;
        index++;
    }

    if (clientCon->dev->glamor || clientCon->dev->nvidia)
    {
        /* copy vmem to smem */
        if (!rdpCopyBoxList(clientCon, clientCon->dev->screenSwPixmap,
                            *out_rects, *num_out_rects))
        {
            return FALSE;
        }
    }

    src = id->pixels;
    dst = id->shmem_pixels;
    dst_format = clientCon->rdp_format;
    src_stride = id->lineBytes;
    dst_stride = clientCon->cap_stride_bytes;

    if (dst_format == XRDP_a8b8g8r8)
    {
        src_bytespp = 4;
        dst_bytespp = 4;

        for (index = 0; index < num_rects; index++)
        {
            /* get rect to copy */
            rect = (*out_rects)[index];

            /* get rect dimensions */
            width = rect.x2 - rect.x1;
            height = rect.y2 - rect.y1;

            /* point to start of each rect in respective memory */
            src_offset = rect.y1 * src_stride + rect.x1 * src_bytespp;
            dst_offset = rect.y1 * dst_stride + rect.x1 * dst_bytespp;
            src_rect = src + src_offset;
            dst_rect = dst + dst_offset;

            /* copy one line at a time */
            for (jndex = 0; jndex < height; jndex++)
            {
                s32 = (const uint32_t *) src_rect;
                d32 = (uint32_t *) dst_rect;
                for (kndex = 0; kndex < width; kndex++)
                {
                    SPLITCOLOR32(red, green, blue, *s32);
                    *d32 = COLOR24(red, green, blue);
                    s32++;
                    d32++;
                }
                src_rect += src_stride;
                dst_rect += dst_stride;
            }
        }
    }
    else
    {
        LLOGLN(0, ("rdpCapture1: unimplemented color conversion"));
    }
    return rv;
}

/******************************************************************************/
static Bool
rdpCapture2(rdpClientCon *clientCon, RegionPtr in_reg, BoxPtr *out_rects,
            int *num_out_rects, struct image_data *id)
{
    int x;
    int y;
    int out_rect_index;
    int num_rects;
    int rcode;
    BoxRec rect;
    BoxRec extents_rect;
    BoxPtr rects;
    RegionRec tile_reg;
    const uint8_t *src;
    uint8_t *dst;
    uint8_t *crc_dst;
    int src_stride;
    int dst_stride;
    int crc_offset;
    int crc_stride;
    uint64_t crc;
    int num_crcs;
    int num_skips;
    int tile_row_stride, crc_height;
    uint64_t *row_hashes;

    LLOGLN(10, ("rdpCapture2:"));

    if (clientCon->shmemstatus != SHM_RFX_ACTIVE) {
        LLOGLN(0, ("rdpCapture2: WARNING -- Shared memory is not configured for RFX. Aborting capture!"));
        return FALSE;
    }

    if (clientCon->dev->glamor || clientCon->dev->nvidia)
    {
        /* copy vmem to smem */
        if (!rdpCopyBoxList(clientCon, clientCon->dev->screenSwPixmap,
                            REGION_RECTS(in_reg), REGION_NUM_RECTS(in_reg)))
        {
            return FALSE;
        }
    }

    *out_rects = g_new(BoxRec, RDP_MAX_TILES);
    if (*out_rects == NULL)
    {
        return FALSE;
    }
    out_rect_index = 0;
    extents_rect = *rdpRegionExtents(in_reg);

    src = id->pixels;
    dst = id->shmem_pixels;
    src_stride = id->lineBytes;
    dst_stride = clientCon->cap_stride_bytes;

    crc_stride = (clientCon->dev->width + 63) / 64;
    /* tile rows are column-major */
    tile_row_stride = ((clientCon->dev->height + 63) / 64) * 64;
    crc_height = ((clientCon->dev->height + 63) / 64);
    num_crcs = crc_stride * crc_height;
    if (num_crcs != clientCon->num_rfx_crcs_alloc)
    {
        /* resize the hash list */
        clientCon->num_rfx_crcs_alloc = num_crcs;
        free(clientCon->rfx_crcs);
        free(clientCon->rfx_tile_row_hashes);
        clientCon->rfx_crcs = g_new0(uint64_t, num_crcs);
        clientCon->rfx_tile_row_hashes = g_new0(uint64_t, num_crcs * 64);
    }

    /* update the tile row hashes. Column major order to be kind to 
       prefetchers even though it shouldn't matter much */
    x = extents_rect.x1 & ~63;
    while (x < extents_rect.x2)
    {
        y = extents_rect.y1 & ~63;
        while (y < extents_rect.y2)
        {
            rect.x1 = x;
            rect.y1 = y;
            rect.x2 = rect.x1 + 64;
            rect.y2 = rect.y1 + 64;
            rcode = rdpRegionContainsRect(in_reg, &rect);
            if (rcode == rgnIN)
            {
                row_hashes = clientCon->rfx_tile_row_hashes + 
                    (x / 64) * tile_row_stride + y;
                wyhash_rfx_tile_rows(src, src_stride, x, y, row_hashes, 64);
            }
            y += 64;
        }
        x += 64;
    }

    y = extents_rect.y1 & ~63;
    num_skips = 0;
    while (y < extents_rect.y2)
    {
        x = extents_rect.x1 & ~63;
        while (x < extents_rect.x2)
        {
            rect.x1 = x;
            rect.y1 = y;
            rect.x2 = rect.x1 + 64;
            rect.y2 = rect.y1 + 64;
            rcode = rdpRegionContainsRect(in_reg, &rect);
            LLOGLN(10, ("rdpCapture2: rcode %d", rcode));

            if (rcode != rgnOUT)
            {
                /* hex digits of pi as a 64 bit int */
                if (rcode == rgnPART)
                {
                    LLOGLN(10, ("rdpCapture2: rgnPART"));
                    rdpFillBox_yuvalp(x, y, dst, dst_stride);
                    rdpRegionInit(&tile_reg, &rect, 0);
                    rdpRegionIntersect(&tile_reg, in_reg, &tile_reg);
                    rects = REGION_RECTS(&tile_reg);
                    num_rects = REGION_NUM_RECTS(&tile_reg);
                    crc = wyhash((const void*)rects, num_rects * sizeof(BoxRec), 
                        crc, _wyp);
                    rdpCopyBox_a8r8g8b8_to_yuvalp(x, y,
                                                  src, src_stride,
                                                  dst, dst_stride,
                                                  rects, num_rects);
                    crc_dst = dst + (y << 8) * (dst_stride >> 8) + (x << 8);
                    crc = 0x3243f6a8885a308dull;
                    crc = wyhash((const void*)crc_dst, 64 * 64 * 4, crc, _wyp);
                    rdpRegionUninit(&tile_reg);
                }
                else /* rgnIN */
                {
                    LLOGLN(10, ("rdpCapture2: rgnIN"));
                    crc = wyhash_rfx_tile_from_rows(
                        clientCon->rfx_tile_row_hashes, tile_row_stride, x, y);

                }
                crc_offset = (y / 64) * crc_stride + (x / 64);
                if (crc == clientCon->rfx_crcs[crc_offset])
                {
                    LLOGLN(10, ("rdpCapture2: crc skip at x: %d, y: %d, "
                                "skip count: %d", x, y, num_skips));
                    num_skips += 1;
                }
                else
                {
                    clientCon->rfx_crcs[crc_offset] = crc;
                    /* lazily only do this if hash wasn't identical */
                    if (rcode != rgnPART)
                    {
                        rdpCopyBox_a8r8g8b8_to_yuvalp(x, y,
                                src, src_stride,
                                dst, dst_stride,
                                &rect, 1);
                    }
                    (*out_rects)[out_rect_index] = rect;
                    out_rect_index++;
                    if (out_rect_index >= RDP_MAX_TILES)
                    {
                        free(*out_rects);
                        *out_rects = NULL;
                        return FALSE;
                    }
                }
            }
            x += 64;
        }
        y += 64;
    }
    *num_out_rects = out_rect_index;
    return TRUE;
}


/******************************************************************************/
/* make out_rects always multiple of 2 width and height */
static Bool
rdpCapture3(rdpClientCon *clientCon, RegionPtr in_reg, BoxPtr *out_rects,
            int *num_out_rects, struct image_data *id)
{
    BoxPtr psrc_rects;
    BoxRec rect;
    int num_rects;
    int num_out_rects_index;
    int num_rects_index;
    BoxPtr lout_rects;
    uint8_t *dst_uv;
    Bool rv;
    const uint8_t *src;
    uint8_t *dst;
    int src_stride;
    int dst_stride;
    int dst_format;

    LLOGLN(10, ("rdpCapture3:"));

    if (!isShmStatusActive(clientCon->shmemstatus)) {
        LLOGLN(0, ("rdpCapture3: WARNING -- Shared memory is not configured. Aborting capture!"));
        return FALSE;
    }

    rv = TRUE;

    num_rects = REGION_NUM_RECTS(in_reg);
    psrc_rects = REGION_RECTS(in_reg);

    if (num_rects < 1)
    {
        return FALSE;
    }

    num_rects = 1;

    lout_rects = g_new(BoxRec, num_rects * 4);
    num_out_rects_index = 0;
    for (num_rects_index = 0; num_rects_index < num_rects; ++num_rects_index)
    {
        rect = psrc_rects[num_rects_index];
        LLOGLN(10, ("old x1 %d y1 %d x2 %d y2 %d", rect.x1, rect.y1,
               rect.x2, rect.y2));
        struct xrdp_rectangle rect_buffer;
        rect_buffer.x = rect.x1;
        rect_buffer.y = rect.y1;
        rect_buffer.width = rect.x2 - rect.x1;
        rect_buffer.height = rect.y2 - rect.y1;
        snapRectangleToGrid(&rect_buffer);
        rect.x1 = max(0, rect_buffer.x);
        rect.y1 = max(0, rect_buffer.y);
        rect.x2 = min(rect_buffer.x + rect_buffer.width, clientCon->dev->width - 1);
        rect.y2 = min(rect_buffer.y + rect_buffer.height, clientCon->dev->height - 1);

        LLOGLN(10, ("new x1 %d y1 %d x2 %d y2 %d\n", rect.x1, rect.y1,
                rect.x2, rect.y2));
        if ((rect.x2 > rect.x1) && (rect.y2 > rect.y1))
        {
            (lout_rects)[num_out_rects_index] = rect;
            ++num_out_rects_index;
        }
    }

    num_rects = num_out_rects_index;
    if (num_rects < 1)
    {
        free(lout_rects);
        return FALSE;
    }
    *out_rects = lout_rects;
    *num_out_rects = num_rects;

    if (clientCon->helperPixmaps[0] != NULL)
    {
        /* copy vmem to vmem */
        rv = rdpCopyBoxList(clientCon, clientCon->helperPixmaps[0],
                            *out_rects, *num_out_rects);
        id->flags = ENCODE_COMPLETE;
        return rv;
        /* helper will do the rest */
    }
    else if (clientCon->dev->glamor || clientCon->dev->nvidia)
    {
        /* copy vmem to smem */
        rv = rdpCopyBoxList(clientCon, clientCon->dev->screenSwPixmap,
                            *out_rects, *num_out_rects);
    }

    src = id->pixels;
    dst = id->shmem_pixels;
    dst_format = clientCon->rdp_format;
    src_stride = id->lineBytes;
    dst_stride = clientCon->cap_stride_bytes;

    if (dst_format == XRDP_a8r8g8b8)
    {
        rdpCopyBox_a8r8g8b8_to_a8r8g8b8(clientCon,
                                        src, src_stride, 0, 0,
                                        dst, dst_stride, 0, 0,
                                        *out_rects, num_rects);
    }
    else if (dst_format == XRDP_yuv444_709fr)
    {
        int full_size = clientCon->cap_width * clientCon->cap_height;
        rdpCopyBox_yuv444_to_streamV2(clientCon,
                                src, src_stride,
                                dst, dst_stride,
                                dst + full_size, dst_stride,
                                dst + full_size * 3 / 2, dst_stride,
                                dst + full_size * 5 / 2, dst_stride,
                                0, 0,
                                0, 0,
                                *out_rects, num_rects,
                                clientCon->cap_width, clientCon->cap_height);
        id->flags |= CONTAINS_DUAL_FRAME_AVC444;
    }
    else if (dst_format == XRDP_nv12_709fr)
    {
        dst_uv = dst;
        dst_uv += clientCon->cap_width * clientCon->cap_height;
        rdpCopyBox_a8r8g8b8_to_nv12_709fr(clientCon,
                                          src, src_stride, 0, 0,
                                          dst, dst_stride,
                                          dst_uv, dst_stride,
                                          0, 0,
                                          *out_rects, num_rects);
    }
    else if (dst_format == XRDP_nv12)
    {
        dst_uv = dst;
        dst_uv += clientCon->cap_width * clientCon->cap_height;
        rdpCopyBox_a8r8g8b8_to_nv12(clientCon,
                                    src, src_stride, 0, 0,
                                    dst, dst_stride,
                                    dst_uv, dst_stride,
                                    0, 0,
                                    *out_rects, num_rects);
    }
    else
    {
        LLOGLN(0, ("rdpCapture3: unimplemented color conversion"));
    }

    return rv;
}

/**
 * Copy an array of rectangles from one memory area to another
 *****************************************************************************/
Bool
rdpCapture(rdpClientCon *clientCon, RegionPtr in_reg, BoxPtr *out_rects,
           int *num_out_rects, struct image_data *id)
{
    int mode;

    mode = clientCon->client_info.capture_code;
    switch (mode)
    {
        case 0:
            return rdpCapture0(clientCon, in_reg, out_rects, num_out_rects, id);
        case 1:
            return rdpCapture1(clientCon, in_reg, out_rects, num_out_rects, id);
        case 2:
            /* used for remotefx capture */
            return rdpCapture2(clientCon, in_reg, out_rects, num_out_rects, id);
        case 3:
            /* used for even align capture */
            return rdpCapture3(clientCon, in_reg, out_rects, num_out_rects, id);
        default:
            LLOGLN(0, ("rdpCapture: mode %d not implemented", mode));
            break;
    }
    return FALSE;
}
