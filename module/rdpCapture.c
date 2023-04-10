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

// #define YUV444_SET_Y(pixel, Y) pixel = ((pixel) & 0xFFFFFF00) | ((Y) << 0)
// #define YUV444_SET_U(pixel, U) pixel = ((pixel) & 0xFFFF00FF) | ((U) << 8)
// #define YUV444_SET_V(pixel, V) pixel = ((pixel) & 0xFF00FFFF) | ((V) << 16)

#define YUV444_SET_V(pixel, V) pixel = (((pixel) & 0xFFFFFF00) | ((V) << 0))
#define YUV444_SET_U(pixel, U) pixel = (((pixel) & 0xFFFF00FF) | ((U) << 8))
#define YUV444_SET_Y(pixel, Y) pixel = (((pixel) & 0xFF00FFFF) | ((Y) << 16))

// #define YUV444_SET_U(pixel, U) pixel = ((pixel) & 0x00FFFFFF) | ((U) << 24)
// #define YUV444_SET_V(pixel, V) pixel = ((pixel) & 0xFF00FFFF) | ((V) << 16)
// #define YUV444_SET_Y(pixel, Y) pixel = ((pixel) & 0xFFFF00FF) | ((Y) << 8)

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
/* copy rects with no error checking */
static int
rdpCopyBox_a8r8g8b8_to_yuv444_709fr(rdpClientCon *clientCon,
                                    const uint8_t *src, int src_stride,
                                    int srcx, int srcy,
                                    uint8_t *dst, int dst_stride,
                                    int dstx, int dsty,
                                    BoxPtr rects, int num_rects)
{
    const uint8_t *s8;
    uint8_t *d8;
    int index;
    int width;
    int height;
    BoxPtr box;

    for (index = 0; index < num_rects; index++)
    {
        box = rects + index;
        s8 = src + (box->y1 - srcy) * src_stride;
        s8 += (box->x1 - srcx) * 4;
        d8 = dst + (box->y1 - dsty) * dst_stride;
        d8 += (box->x1 - dstx) * 4;
        width = box->x2 - box->x1;
        height = box->y2 - box->y1;
        a8r8g8b8_to_yuv444_709fr_box(s8, src_stride,
                                     d8, dst_stride,
                                     width, height);
    }
    return 0;
}

//

#define XY_BYTE_COORDINATE(x, y, stride, bytes_per_cell) \
    ((x) * bytes_per_cell) + ((y) * stride)

#define XY_BYTE_COORDINATE_HALF(x, y, stride, bytes_per_cell) \
    ((x) * bytes_per_cell) + ((y) * stride)

// #define YUV444_GET_V(pixel) 
//     (pixel >> 16) & UCHAR_MAX
// #define YUV444_GET_U(pixel) 
//     (pixel >> 8) & UCHAR_MAX
// #define YUV444_GET_Y(pixel) 
//     (pixel >> 0) & UCHAR_MAX

// #define YUV444_GET_Y(pixel)
//     (pixel >> 16) & UCHAR_MAX
// #define YUV444_GET_U(pixel)
//     (pixel >> 8) & UCHAR_MAX
// #define YUV444_GET_V(pixel)
//     (pixel >> 0) & UCHAR_MAX

// RGB_SPLIT(R, G, B, pixel);
// Y =  ( 54 * R + 183 * G +  18 * B) >> 8;
// U = ((-29 * R -  99 * G + 128 * B) >> 8) + 128;
// V = ((128 * R - 116 * G -  12 * B) >> 8) + 128;
// cY = RDPCLAMP(Y, 0, UCHAR_MAX) << 16;
// cU = RDPCLAMP(U, 0, UCHAR_MAX) << 8;
// cV = RDPCLAMP(V, 0, UCHAR_MAX);
// d32[0] = cY | cU | cV;

// static INLINE BYTE RGB2Y(BYTE R, BYTE G, BYTE B)
// {
// 	return (54 * R + 183 * G + 18 * B) >> 8;
// }

// static INLINE BYTE RGB2U(BYTE R, BYTE G, BYTE B)
// {
// 	return ((-29u * R - 99u * G + 128u * B) >> 8u) + 128u;
// }

// static INLINE BYTE RGB2V(INT32 R, INT32 G, INT32 B)
// {
// 	return ((128lu * R - 116lu * G - 12lu * B) >> 8lu) + 128lu;
// }

static int
extractY(const uint8_t *image_data, int x, int y, int width, int height, 
         int image_stride, int bytes_per_pixel, uint8_t *value) {
    const int offset = XY_BYTE_COORDINATE(x, y, image_stride, bytes_per_pixel);
    const uint32_t *pixel = (const uint32_t*)(image_data + offset);
    uint32_t R, G, B, Y;
    SPLITCOLOR32(R, G, B, *pixel);
    Y = (54 * R + 183 * G + 18 * B) >> 8;
    *value = (uint8_t)RDPCLAMP(Y, 0, UCHAR_MAX);
    return 0;
}

static int
extractU(const uint8_t *image_data, int x, int y, int width, int height, 
         int image_stride, int bytes_per_pixel, uint8_t *value) {
    const int offset = XY_BYTE_COORDINATE(x, y, image_stride, bytes_per_pixel);
    const uint32_t *pixel = (const uint32_t*)(image_data + offset);
    uint32_t R, G, B, U;
    SPLITCOLOR32(R, G, B, *pixel);
    U = ((-29u * R - 99u * G + 128u * B) >> 8u) + 128u;
    *value = (uint8_t)RDPCLAMP(U, 0, UCHAR_MAX);
    return 0;
}

static int
extractV(const uint8_t *image_data, int x, int y, int width, int height, 
         int image_stride, int bytes_per_pixel, uint8_t *value) {
    const int offset = XY_BYTE_COORDINATE(x, y, image_stride, bytes_per_pixel);
    const uint32_t *pixel = (const uint32_t*)(image_data + offset);
    uint32_t R, G, B, V;
    SPLITCOLOR32(R, G, B, *pixel);
    V = ((128lu * R - 116lu * G - 12lu * B) >> 8lu) + 128lu;
    *value = (uint8_t)RDPCLAMP(V, 0, UCHAR_MAX);
    return 0;
}

// static int general_YUV444SplitToYUV420(const uint8_t *pSrc, int src_stride,
//                                              uint8_t *pMainDst, int dst_main_stride,
//                                              uint8_t *pAuxDst, int dst_aux_stride,
//                                              int full_width, int full_height)
// {
//     LLOGLN(0, ("split it!"));
// 	uint32_t x, y, uY = 0, vY = 0;
// 	uint32_t halfWidth, halfHeight;
// 	/* The auxilary frame is aligned to multiples of 16x16.
// 	 * We need the padded height for B4 and B5 conversion. */
// 	const uint32_t padHeight = full_height + 16 - full_height % 16;
//     uint32_t *yuv_dst_buffer;
//     uint32_t extracted_value;
// 	halfWidth = (full_width + 1) / 2;
// 	halfHeight = (full_height + 1) / 2;

// 	/* B1 */
// 	for (y = 0; y < full_height; ++y)
// 	{
//         for (x = 0; x < full_width; ++x)
// 	    {
//             //LLOGLN(0, ("B1 (x, y): %d, %d", y));
//             yuv_dst_buffer = (uint32_t*)(dst_main + XY_BYTE_COORDINATE(x, y, dst_main_stride, BYTES_PER_CELL));
//             if (extractY(pSrc, x, y, full_width, full_height, src_stride, BYTES_PER_CELL, &extracted_value)) {
//                 continue;
//             }
//             YUV444_SET_Y(*yuv_dst_buffer, extracted_value);
//         }
// 	}

//     LLOGLN(0, ("no crash 1"));

// 	/* B2 and B3 */
// 	for (y = 0; y < halfHeight; ++y)
// 	{
// 		for (x = 0; x < halfWidth; x++)
// 		{
//             uint32_t one, two, three, four;

//             yuv_dst_buffer = (uint32_t*)(dst_main + XY_BYTE_COORDINATE(x, y, dst_main_stride, BYTES_PER_CELL));

//             if (!extractU(s8, 2 * x, 2 * y, full_width, full_height, src_stride, BYTES_PER_CELL, &one) &&
//                 !extractU(s8, 2 * x + 1, 2 * y, full_width, full_height, src_stride, BYTES_PER_CELL, &two) &&
//                 !extractU(s8, 2 * x, 2 * y + 1, full_width, full_height, src_stride, BYTES_PER_CELL, &three) &&
//                 !extractU(s8, 2 * x + 1, 2 * y + 1, full_width, full_height, src_stride, BYTES_PER_CELL, &four)
//                 )
//             {
//                 YUV444_SET_U(*yuv_dst_buffer, RDPCLAMP((one + two + three + four) / 4, 0, UCHAR_MAX));
//             }

//             if (!extractV(s8, 2 * x, 2 * y, mod_width, mod_height, src_stride, BYTES_PER_CELL, &one) &&
//                 !extractV(s8, 2 * x + 1, 2 * y, mod_width, mod_height, src_stride, BYTES_PER_CELL, &two) &&
//                 !extractV(s8, 2 * x, 2 * y + 1, mod_width, mod_height, src_stride, BYTES_PER_CELL, &three) &&
//                 !extractV(s8, 2 * x + 1, 2 * y + 1, mod_width, mod_height, src_stride, BYTES_PER_CELL, &four)
//                 )
//             {
//                 YUV444_SET_V(*yuv_dst_buffer, RDPCLAMP((one + two + three + four) / 4, 0, UCHAR_MAX));
//             }
// 		}
// 	}

// 	/* B4 and B5 */
// 	for (y = 0; y < padHeight; ++y)
// 	{
// 		uint8_t* pY = pAuxDst[0] + y * dst_aux_stride;

// 		if (y % 16 < 8)
// 		{
// 			const uint32_t pos = (2 * uY++ + 1);
// 			const uint8_t* pSrcU = pSrc[1] + pos * src_stride;

// 			if (pos >= full_height)
// 				continue;

// 			memcpy(pY, pSrcU, full_width);
// 		}
// 		else
// 		{
// 			const uint32_t pos = (2 * vY++ + 1);
// 			const uint8_t* pSrcV = pSrc[2] + pos * src_stride;

// 			if (pos >= full_height)
// 				continue;

// 			memcpy(pY, pSrcV, full_width);
// 		}
// 	}

// 	/* B6 and B7 */
// 	for (y = 0; y < halfHeight; ++y)
// 	{
// 		const uint8_t* pSrcU = pSrc[1] + 2 * y * src_stride;
// 		const uint8_t* pSrcV = pSrc[2] + 2 * y * src_stride;
// 		uint8_t* pU = pAuxDst[1] + y * dst_aux_stride;
// 		uint8_t* pV = pAuxDst[2] + y * dst_aux_stride;

// 		for (x = 0; x < halfWidth; x++)
// 		{
// 			pU[x] = pSrcU[2 * x + 1];
// 			pV[x] = pSrcV[2 * x + 1];
// 		}
// 	}

// 	return 0;
// }

/******************************************************************************/
int
a8r8g8b8_to_yuv444_709fr_box_streamV2_cmp(const uint8_t *s8, int src_stride,
                                      uint8_t *dst_main_y, int dst_main_y_stride,
                                      uint8_t *dst_main_u, int dst_main_u_stride,
                                      uint8_t *dst_main_v, int dst_main_v_stride,
                                      uint8_t *dst_aux_y, int dst_aux_y_stride,
                                      uint8_t *dst_aux_u, int dst_aux_u_stride,
                                      uint8_t *dst_aux_v, int dst_aux_v_stride,
                                      int full_width, int full_height)
{
    const int src_bytes_per_pixel = 4;
    int half_width = (full_width + 1) / 2;
    int half_height = (full_height + 1) / 2;
    int quarter_width = (full_width + 1) / 4;
    int quarter_height = (full_height + 1) / 4;
    uint32_t x, y;
    uint8_t *yuv_dst_buffer;
    uint8_t extracted_value;

    //LLOGLN(0, ("initial"));

    for (y = 0; y < full_height; ++y)
    {
        for (x = 0; x < full_width; ++x)
        {
            // B1[x, y] = Y444[x, y]
            //LLOGLN(0, ("Testing y 444: x %d y %d", x, y));
            yuv_dst_buffer = dst_main_y + XY_BYTE_COORDINATE(x, y, dst_main_y_stride, 1);
            if (extractY(s8, x, y, full_width, full_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                continue;
            }
            yuv_dst_buffer[0] = (uint8_t)extracted_value;
        }
    }

    //LLOGLN(0, ("finished main Y frame"));

    for (y = 0; y < half_height; ++y)
    {
        for (x = 0; x < half_width; ++x)
        {
            uint8_t one, two, three, four;

            //LLOGLN(0, ("Pre-extract main U"));
            // B2[x, y] = U444[2 * x, 2 * y];
            yuv_dst_buffer = dst_main_u + XY_BYTE_COORDINATE_HALF(x, y, dst_main_u_stride, 1);
            if (!extractU(s8, 2 * x, 2 * y, full_width, full_height, src_stride, src_bytes_per_pixel, &one) &&
                !extractU(s8, 2 * x + 1, 2 * y, full_width, full_height, src_stride, src_bytes_per_pixel, &two) &&
                !extractU(s8, 2 * x, 2 * y + 1, full_width, full_height, src_stride, src_bytes_per_pixel, &three) &&
                !extractU(s8, 2 * x + 1, 2 * y + 1, full_width, full_height, src_stride, src_bytes_per_pixel, &four)
                )
            {
                //LLOGLN(0, ("Actual-extract U main UV"));
                yuv_dst_buffer[0] = (uint8_t)RDPCLAMP(((int)one + two + three + four + 2) / 4, 0, UCHAR_MAX);
            }

            //LLOGLN(0, ("Pre-extract main V"));
            // B3[x, y] = V444[2 * x, 2 * y];
            yuv_dst_buffer = dst_main_v + XY_BYTE_COORDINATE_HALF(x, y, dst_main_v_stride, 1);
            if (!extractV(s8, 2 * x, 2 * y, full_width, full_height, src_stride, src_bytes_per_pixel, &one) &&
                !extractV(s8, 2 * x + 1, 2 * y, full_width, full_height, src_stride, src_bytes_per_pixel, &two) &&
                !extractV(s8, 2 * x, 2 * y + 1, full_width, full_height, src_stride, src_bytes_per_pixel, &three) &&
                !extractV(s8, 2 * x + 1, 2 * y + 1, full_width, full_height, src_stride, src_bytes_per_pixel, &four)
                )
            {
                //LLOGLN(0, ("Actual-extract V main UV"));
                yuv_dst_buffer[0] = (uint8_t)RDPCLAMP(((int)one + two + three + four + 2) / 4, 0, UCHAR_MAX);
            }
        }
    }

    LLOGLN(0, ("finished main UV frame"));

    ////////////////////////// END MAIN VIEW ///////////////////////////////////

    for (y = 0; y < full_height; ++y)
    {
        for (x = 0; x < half_width; ++x)
        {
            // B4[x, y] = U444[2x + 1, y];
            yuv_dst_buffer = dst_aux_y + XY_BYTE_COORDINATE_HALF(x, y, dst_aux_y_stride, 1);
            if (!extractU(s8, 2 * x + 1, y, full_width, full_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                yuv_dst_buffer[0] = (uint8_t)extracted_value;
            }

            // B5[(W/2) + x, y] = V444[2x + 1, y];
            yuv_dst_buffer = dst_aux_y + XY_BYTE_COORDINATE_HALF(half_width + x, y, dst_aux_y_stride, 1);
            if (!extractV(s8, 2 * x + 1, y, full_width, full_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                yuv_dst_buffer[0] = (uint8_t)extracted_value;
            }
        }
    }

    for (y = 0; y < half_height; ++y)
    {
        for (x = 0; x < quarter_width; ++x)
        {
            //LLOGLN(0, ("Pre-extract aux U1"));
            // B6[x, y] = U444[4 * x, 2 * y + 1];
            yuv_dst_buffer = dst_aux_u + XY_BYTE_COORDINATE_HALF(x, y, dst_aux_u_stride, 1);
            if (!extractU(s8, 4 * x, 2 * y + 1, full_width, full_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                yuv_dst_buffer[0] = (uint8_t)extracted_value;
            }

            //LLOGLN(0, ("Pre-extract aux V1"));
            // B7[(W/4) + x, y] = V444[4 * x, 2 * y + 1];
            yuv_dst_buffer = dst_aux_u + XY_BYTE_COORDINATE_HALF(quarter_width + x, y, dst_aux_u_stride, 1);
            if (!extractV(s8, 4 * x, 2 * y + 1, full_width, full_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                yuv_dst_buffer[0] = (uint8_t)extracted_value;
            }

            //LLOGLN(0, ("Pre-extract aux U2"));
            // B8[x, y] = U444[4 * x + 2, 2 * y + 1];
            yuv_dst_buffer = dst_aux_v + XY_BYTE_COORDINATE_HALF(x, y, dst_aux_v_stride, 1);
            if (!extractU(s8, 4 * x + 2, 2 * y + 1, full_width, full_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                yuv_dst_buffer[0] = (uint8_t)extracted_value;
            }

            //LLOGLN(0, ("Pre-extract aux V2"));
            // B9[(W/4) + x, y] = V444[4 * x + 2, 2 * y + 1];
            yuv_dst_buffer = dst_aux_v + XY_BYTE_COORDINATE_HALF(quarter_width + x, y, dst_aux_v_stride, 1);
            if (!extractV(s8, 4 * x + 2, 2 * y + 1, full_width, full_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
                yuv_dst_buffer[0] = (uint8_t)extracted_value;
            }
        }
    }

    // uint8_t *a, *temp_a = malloc(full_height * full_width * dst_aux_uv_stride);
    // uint8_t *b, *temp_b = malloc(full_height * full_width * dst_aux_uv_stride);
    // a = temp_a;
    // b = temp_b;
    // int temp_size = 0;

    // for (y = 0; y < half_height; ++y)
    // {
    //     for (x = 0; x < quarter_width; ++x)
    //     {
    //         // B6[x, y] = U444[4 * x, 2 * y + 1];
    //         yuv_dst_buffer = temp_a + XY_BYTE_COORDINATE(x, y, 1, 1);
    //         if (!extractU(s8, 4 * x, 2 * y + 1, full_width, full_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
    //             yuv_dst_buffer[0] = (uint8_t)extracted_value;
    //         }

    //         // B7[(W/4) + x, y] = V444[4 * x, 2 * y + 1];
    //         yuv_dst_buffer = temp_a + XY_BYTE_COORDINATE(quarter_width + x, y, 1, 1);
    //         if (!extractU(s8, 4 * x, 2 * y + 1, full_width, full_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
    //             yuv_dst_buffer[0] = (uint8_t)extracted_value;
    //         }

    //         // B8[x, y] = U444[4 * x + 2, 2 * y + 1];
    //         yuv_dst_buffer = temp_b + XY_BYTE_COORDINATE(x, y, 1, 1);
    //         if (!extractV(s8, 4 * x + 2, 2 * y + 1, full_width, full_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
    //             yuv_dst_buffer[0] = (uint8_t)extracted_value;
    //         }

    //         // B9[(W/4) + x, y] = V444[4 * x + 2, 2 * y + 1];
    //         yuv_dst_buffer = temp_b + XY_BYTE_COORDINATE(quarter_width + x, y, 1, 1);
    //         if (!extractV(s8, 4 * x + 2, 2 * y + 1, full_width, full_height, src_stride, src_bytes_per_pixel, &extracted_value)) {
    //             yuv_dst_buffer[0] = (uint8_t)extracted_value;
    //         }
    //         ++temp_size;
    //     }
    // }

    // // LLOGLN(0, ("Testing"));

    // for (x = 0; x < temp_size; ++x) {
    //     if (x % 2 == 0) { //even
    //         *(dst_aux_uv + x) = temp_a[0];
    //         ++temp_a;
    //     }
    //     else
    //     {
    //         *(dst_aux_uv + x) = temp_b[0];
    //         ++temp_b;
    //     }
    // }

    // free(a);
    // free(b);

    return 0;
}

uint8_t RGB2Y(uint8_t R, uint8_t G, uint8_t B)
{
	return (54 * R + 183 * G + 18 * B) >> 8;
}

uint8_t RGB2U(uint8_t R, uint8_t G, uint8_t B)
{
	return ((-29u * R - 99u * G + 128u * B) >> 8u) + 128u;
}

uint8_t RGB2V(uint8_t R, uint8_t G, uint8_t B)
{
	return ((128lu * R - 116lu * G - 12lu * B) >> 8lu) + 128lu;
}

static void general_RGBToAVC444YUVv2_ANY_DOUBLE_ROW(
    const uint8_t* srcEven, const uint8_t* srcOdd, uint8_t* yLumaDstEven,
    uint8_t* yLumaDstOdd, uint8_t* uLumaDst, uint8_t* vLumaDst, uint8_t* yEvenChromaDst1, uint8_t* yEvenChromaDst2,
    uint8_t* yOddChromaDst1, uint8_t* yOddChromaDst2, uint8_t* uChromaDst1, uint8_t* uChromaDst2,
    uint8_t* vChromaDst1, uint8_t* vChromaDst2, uint32_t width)
{
	uint32_t x;
	const uint32_t bpp = 4;

	for (x = 0; x < width; x += 2)
	{
		uint8_t Ya, Ua, Va;
		uint8_t Yb, Ub, Vb;
		uint8_t Yc, Uc, Vc;
		uint8_t Yd, Ud, Vd;
		{
			BYTE b, g, r;
			const int color = *((uint32_t*)srcEven);
			srcEven += bpp;
			SPLITCOLOR32(r, g, b, color);
			Ya = RGB2Y(r, g, b);
			Ua = RGB2U(r, g, b);
			Va = RGB2V(r, g, b);
		}

		if (x < width - 1)
		{
			BYTE b, g, r;
			const int color = *((uint32_t*)srcEven);
			srcEven += bpp;
			SPLITCOLOR32(r, g, b, color);
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
			BYTE b, g, r;
			const int color = *((uint32_t*)srcOdd);
			srcOdd += bpp;
			SPLITCOLOR32(r, g, b, color);
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

		if (srcOdd && (x < width - 1))
		{
			BYTE b, g, r;
			const int color = *((uint32_t*)srcOdd);
			srcOdd += bpp;
			SPLITCOLOR32(r, g, b, color);
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

		if (x < width - 1)
			*yLumaDstEven++ = Yb;

		if (srcOdd)
			*yLumaDstOdd++ = Yc;

		if (srcOdd && (x < width - 1))
			*yLumaDstOdd++ = Yd;

		/* 2x 2y [b2,b3] */
		*uLumaDst++ = (Ua + Ub + Uc + Ud) / 4;
		*vLumaDst++ = (Va + Vb + Vc + Vd) / 4;

		/* 2x+1, y [b4,b5] even */
		if (x < width - 1)
		{
			*yEvenChromaDst1++ = Ub;
			*yEvenChromaDst2++ = Vb;
		}

		if (srcOdd)
		{
			/* 2x+1, y [b4,b5] odd */
			if (x < width - 1)
			{
				*yOddChromaDst1++ = Ud;
				*yOddChromaDst2++ = Vd;
			}

			/* 4x 2y+1 [b6, b7] */
			if (x % 4 == 0)
			{
				*uChromaDst1++ = Uc;
				*uChromaDst2++ = Vc;
			}
			/* 4x+2 2y+1 [b8, b9] */
			else
			{
				*vChromaDst1++ = Uc;
				*vChromaDst2++ = Vc;
			}
		}
	}
}

static int general_RGBToAVC444YUVv2_ANY(const uint8_t* pSrc,
                                        uint32_t srcStep, 
                                        uint8_t* pDst1[3],
                                        const uint32_t dst1Step[3], 
                                        uint8_t* pDst2[3],
                                        const uint32_t dst2Step[3],
                                        const uint32_t width,
                                        const uint32_t height)
{
	uint32_t y;

	if (height < 1 || width < 1)
		return 1;

	for (y = 0; y < height; y += 2)
	{
		const uint8_t* srcEven = (pSrc + y * srcStep);
		const uint8_t* srcOdd = (y < height - 1) ? (srcEven + srcStep) : NULL;
		uint8_t* dstLumaYEven = (pDst1[0] + y * dst1Step[0]);
		uint8_t* dstLumaYOdd = (dstLumaYEven + dst1Step[0]);
		uint8_t* dstLumaU = (pDst1[1] + (y / 2) * dst1Step[1]);
		uint8_t* dstLumaV = (pDst1[2] + (y / 2) * dst1Step[2]);
		uint8_t* dstEvenChromaY1 = (pDst2[0] + y * dst2Step[0]);
		uint8_t* dstEvenChromaY2 = dstEvenChromaY1 + width / 2;
		uint8_t* dstOddChromaY1 = dstEvenChromaY1 + dst2Step[0];
		uint8_t* dstOddChromaY2 = dstEvenChromaY2 + dst2Step[0];
		uint8_t* dstChromaU1 = (pDst2[1] + (y / 2) * dst2Step[1]);
		uint8_t* dstChromaV1 = (pDst2[2] + (y / 2) * dst2Step[2]);
		uint8_t* dstChromaU2 = dstChromaU1 + width / 4;
		uint8_t* dstChromaV2 = dstChromaV1 + width / 4;
		general_RGBToAVC444YUVv2_ANY_DOUBLE_ROW(
		    srcEven, srcOdd, dstLumaYEven, dstLumaYOdd, dstLumaU, dstLumaV,
		    dstEvenChromaY1, dstEvenChromaY2, dstOddChromaY1, dstOddChromaY2, dstChromaU1,
		    dstChromaU2, dstChromaV1, dstChromaV2, width);
	}

	return 0;
}

int
a8r8g8b8_to_yuv444_709fr_box_streamV2_freerdp(const uint8_t *s8, int src_stride,
                                      uint8_t *dst_main_y, int dst_main_y_stride,
                                      uint8_t *dst_main_u, int dst_main_u_stride,
                                      uint8_t *dst_main_v, int dst_main_v_stride,
                                      uint8_t *dst_aux_y, int dst_aux_y_stride,
                                      uint8_t *dst_aux_u, int dst_aux_u_stride,
                                      uint8_t *dst_aux_v, int dst_aux_v_stride,
                                      int full_width, int full_height)
{
    // static int general_RGBToAVC444YUVv2_ANY(const uint8_t* pSrc,
    //                                     uint32_t srcFormat,
    //                                     uint32_t srcStep, 
    //                                     uint8_t* pDst1[3],
    //                                     const uint32_t dst1Step[3], 
    //                                     uint8_t* pDst2[3],
    //                                     const uint32_t dst2Step[3],
    //                                     const uint32_t width,
    //                                     const uint32_t height)

    uint8_t* pDst1[3] = {dst_main_y, dst_main_u, dst_main_v};
    uint32_t pStep1[3] = {dst_main_y_stride, dst_main_u_stride, dst_main_v_stride};

    uint8_t* pDst2[3] = {dst_aux_y, dst_aux_u, dst_aux_v};
    uint32_t pStep2[3] = {dst_aux_y_stride, dst_aux_u_stride, dst_aux_v_stride};

    general_RGBToAVC444YUVv2_ANY(
        s8,
        src_stride,
        pDst1, pStep1,
        pDst2, pStep2,
        full_width, full_height
    );
    return 0;
}

/* copy rects with no error checking */
static int
rdpCopyBox_yuv444_to_streamV2(rdpClientCon *clientCon,
                              const uint8_t *src, int src_stride,
                              uint8_t *dst_main_y, int dst_main_y_stride,
                              uint8_t *dst_main_u, int dst_main_u_stride,
                              uint8_t *dst_main_v, int dst_main_v_stride,
                              uint8_t *dst_aux_y, int dst_aux_y_stride,
                              uint8_t *dst_aux_u, int dst_aux_u_stride,
                              uint8_t *dst_aux_v, int dst_aux_v_stride,
                              int srcx, int srcy,
                              int dstx, int dsty,
                              BoxPtr rects, int num_rects)
{
    const uint8_t *s8;
    uint8_t *d8_main_y;
    uint8_t *d8_main_u;
    uint8_t *d8_main_v;
    uint8_t *d8_aux_y;
    uint8_t *d8_aux_u;
    uint8_t *d8_aux_v;
    int index;
    int width;
    int height;
    BoxPtr box;

    // First convert to YUV444
    // for (index = 0; index < num_rects; ++index)
    // {
    //     box = rects + index;
    //     s8 = src + (box->y1 - srcy) * src_stride;
    //     s8 += (box->x1 - srcx) * 4;

    //     d8_main_y = dst_main_y + (box->y1 - dsty) * dst_main_y_stride;
    //     d8_main_y += (box->x1 - dstx) * 1;
    //     d8_main_uv = dst_main_uv + ((box->y1 - dsty) / 2) * dst_main_uv_stride;
    //     d8_main_uv += (box->x1 - dstx) * 2;

    //     width = box->x2 - box->x1;
    //     height = box->y2 - box->y1;


    // }

        // d8_aux_y = dst_aux_y + (box->y1 - dst_aux_coord_y) * dst_aux_y_stride;
        // d8_aux_y += (box->x1 - dst_aux_coord_x) * 3;

        // width = box->x2 - box->x1;
        // height = box->y2 - box->y1;

        // a8r8g8b8_to_yuv444_709fr_box(s8, src_stride,
        //                              d8_main_y, dst_main_y_stride,
        //                              width, height);
        // a8r8g8b8_to_yuv444_709fr_box(s8, src_stride,
        //                              d8_aux_y, dst_aux_y_stride,
        //                              width, height);
    //}

    // Now that it's been converted, split the streams
    // for (index = 0; index < num_rects; ++index)
    // {
    //     box = rects + index;
    //     s8 = src + (box->y1 - srcy) * src_stride;
    //     s8 += (box->x1 - srcx) * 4;

    //     d8_main_y = dst_main_y + (box->y1 - dsty) * dst_main_y_stride;
    //     d8_main_y += (box->x1 - dstx) * 1;
    //     d8_main_uv = dst_main_uv + ((box->y1 - dsty) * 1) * dst_main_uv_stride;
    //     d8_main_uv += (box->x1 - dstx) * 2;

    //     d8_aux_y = dst_aux_y + (box->y1 - dst_aux_coord_y) * dst_aux_y_stride;
    //     d8_aux_y += (box->x1 - dst_aux_coord_x) * 1;
    //     d8_aux_uv = dst_aux_uv + (box->y1 - dst_aux_coord_y) * dst_aux_uv_stride;
    //     d8_aux_uv += (box->x1 - dst_aux_coord_x) * 2;

    //     width = box->x2 - box->x1;
    //     height = box->y2 - box->y1;

        s8 = src;
        d8_main_y = dst_main_y;
        d8_main_u = dst_main_u;
        d8_main_v = dst_main_v;

        d8_aux_y = dst_aux_y;
        d8_aux_u = dst_aux_u;
        d8_aux_v = dst_aux_v;

        width = 1024;
        height = 768;

        // clientCon->dev->a8r8g8b8_to_nv12_709fr_box(src, src_stride,
        //                                            d8_main_y, dst_main_y_stride,
        //                                            d8_main_uv, dst_main_uv_stride,
        //                                            width, height);

        //const int pad_height = RDPCLAMP(height + 16 - height % 16, 0, height);
        //const int pad_width = RDPCLAMP(width + 16 - width % 16, 0, width);

        a8r8g8b8_to_yuv444_709fr_box_streamV2_freerdp(s8, src_stride,
                                              d8_main_y, dst_main_y_stride,
                                              d8_main_u, dst_main_u_stride,
                                              d8_main_v, dst_main_v_stride,
                                              d8_aux_y, dst_aux_y_stride,
                                              d8_aux_u, dst_aux_u_stride,
                                              d8_aux_v, dst_aux_v_stride,
                                              width, height);
    //}
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
    uint8_t *dst_aux;
    int src_stride;
    int dst_stride;
    int dst_format;

    LLOGLN(0, ("rdpCapture3:"));

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

    lout_rects = g_new(BoxRec, num_rects * 4);
    num_out_rects_index = 0;
    for (num_rects_index = 0; num_rects_index < num_rects; num_rects_index++)
    {
        rect = psrc_rects[num_rects_index];
        LLOGLN(10, ("old x1 %d y1 %d x2 %d y2 %d", rect.x1, rect.y1,
               rect.x2, rect.y2));
        rect.x1 -= rect.x1 & 1;
        rect.y1 -= rect.y1 & 1;
        rect.x2 += rect.x2 & 1;
        rect.y2 += rect.y2 & 1;
        /* todo: clip to monitor as well */
        while (rect.x2 > clientCon->dev->width)
        {
            rect.x2 -= 2;
        }
        while (rect.y2 > clientCon->dev->height)
        {
            rect.y2 -= 2;
        }
        if ((rect.x2 > rect.x1) && (rect.y2 > rect.y1))
        {
            (lout_rects)[num_out_rects_index] = rect;
            num_out_rects_index++;
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
        id->flags |= 1;
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
        int full_size = 1024 * 768;
        int half_size = (full_size + 1) / 2;
        int quarter_size = (full_size + 1) / 4;
        int half_stride = (dst_stride + 1) / 2;

        rdpCopyBox_yuv444_to_streamV2(clientCon,
                                      //src
                                      src, src_stride,
                                      //dst_main_y
                                      dst, dst_stride,
                                      //dst_main_u
                                      dst + full_size, half_stride,
                                      //dst_main_v
                                      dst + full_size + quarter_size, half_stride,
                                      //dst_aux_y
                                      dst + full_size + half_size, dst_stride,
                                      //dst_aux_u
                                      dst + 2 * full_size + half_size, half_stride,
                                      //dst_aux_v
                                      dst + 2 * full_size + half_size + quarter_size, half_stride,
                                      0, 0,
                                      0, 0,
                                      *out_rects, num_rects);
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
