/* Copyright 2018 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#ifdef STD_LIB
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#else
#include "arm_math.h"
#include "qfplib-m3.h"
#define expf    qfp_fexp
#endif // STD_LIB

#include "eta_types.h"
#include "print_util.h"
#include "ssd_detection_softnms.h"
#include "FreeRTOS.h"
#include "portable.h"

#ifdef __cplusplus
extern    "C"
{
#endif


//#define HARD_NMS


#define	max(a, b)	((a) >= (b) ? (a) : (b))
#define	min(a, b)	((a) <= (b) ? (a) : (b))

#define areaBox(b)  ( ((int16_t)((b).ymax) - (int16_t)((b).ymin)) * ((int16_t)((b).xmax) - (int16_t)((b).xmin)) )


/*
// Output tensors
const int kOutputTensorDetectionBoxes = 0;
const int kOutputTensorDetectionClasses = 1;
const int kOutputTensorDetectionScores = 2;
const int kOutputTensorNumDetections = 3;
*/

// Object Detection model produces axis-aligned boxes in two formats:
// BoxCorner represents the lower left corner (xmin, ymin) and
// the upper right corner (xmax, ymax).
// CenterSize represents the center (xcenter, ycenter), height and width.
// BoxCornerEncodingFloat and CenterSizeEncoding are related as follows:
// ycenter = y / y_scale * anchor.h + anchor.y;
// xcenter = x / x_scale * anchor.w + anchor.x;
// half_h = 0.5*exp(h/ h_scale)) * anchor.h;
// half_w = 0.5*exp(w / w_scale)) * anchor.w;
// ymin = ycenter - half_h
// ymax = ycenter + half_h
// xmin = xcenter - half_w
// xmax = xcenter + half_w
typedef struct {
    q7_t ymin;
    q7_t xmin;
    q7_t ymax;
    q7_t xmax;
} BoxCornerEncodingFloat;

typedef struct {
    q7_t y;
    q7_t x;
    q7_t h;
    q7_t w;
} CenterSizeEncoding;

/*
typedef struct {
    float y;
    float x;
    float h;
    float w;
} fCenterSizeEncoding;
*/

#if 0
// Returns approximate value of e^x
// using sum of first n terms of Taylor Series
float fexpf( int n, float x )
{
    float sum = 1.0f; // initialize sum of series
    float fi = n - 1.0f;

    for (int i = n - 1; i > 0; --i, fi -= 1.0f )
        sum = 1 + x * sum / fi;

    return sum;
}
#endif

/*
class Dequantizer {
    public:
    Dequantizer(int zero_point, float scale)
	: zero_point_(zero_point), scale_(scale) {}
    float operator()(uint8 x) {
	    return (static_cast<float>(x) - zero_point_) * scale_;
    }

    private:
    int zero_point_;
    float scale_;
};

void DequantizeBoxEncodings( const q7_t* input_box_encodings, int idx,
			     float quant_zero_point, float quant_scale,
			     int length_box_encoding,
			     CenterSizeEncoding* box_centersize )
{
    const uint8* boxes = input_box_encodings + length_box_encoding * idx;
    Dequantizer dequantize( quant_zero_point, quant_scale );

    // See definition of the KeyPointBoxCoder at
    // https://github.com/tensorflow/models/blob/master/research/object_detection/box_coders/keypoint_box_coder.py
    // The first four elements are the box coordinates, which is the same as the
    // FastRnnBoxCoder at
    // https://github.com/tensorflow/models/blob/master/research/object_detection/box_coders/faster_rcnn_box_coder.py
    box_centersize->y = dequantize(boxes[0]);
    box_centersize->x = dequantize(boxes[1]);
    box_centersize->h = dequantize(boxes[2]);
    box_centersize->w = dequantize(boxes[3]);
}

void DequantizeBoxEncodings( const CenterSizeEncoding iLoc,
			                 const float scale,
			                 fCenterSizeEncoding * pfLoc )
{
    pfLoc->y = iLoc.y * scale;
    pfLoc->x = iLoc.x * scale;
    pfLoc->h = iLoc.h * scale;
    pfLoc->w = iLoc.w * scale;
}
*/


q7_t inline add_q7( const q7_t v1, const q7_t dec1, const q7_t v2, const q7_t dec2, const q7_t ret_dec )
{
    int32_t v;
    int8_t  dec = dec1;

    if ( dec1 == dec2 ) {
        v = (int32_t)v1 + (int32_t)v2;
    } else if ( dec1 > dec2 ) {
        v = (int32_t)v1 + ((int32_t)v2 << (dec1 - dec2));
    } else {
        v = ((int32_t)v1 << (dec2 - dec1)) + (int32_t)v2;
        dec = dec2;
    }
    if ( dec > ret_dec )
        v >>= dec - ret_dec;
    else if ( dec < ret_dec )
        v <<= ret_dec - dec;

    return (q7_t) __SSAT( v, 8 );
}

q7_t inline mult_q7( const q7_t v1, const q7_t dec1, const q7_t v2, const q7_t dec2, const q7_t ret_dec )
{
    int32_t v = (int16_t)v1 * (int16_t)v2;
    int8_t  rsh = dec1 + dec2 - ret_dec;

    if ( rsh > 0 ) {
        v >>= rsh;
    } else if ( rsh < 0 ) {
        v <<= (-rsh);
    }

    return (q7_t) __SSAT( v, 8 );
}

q7_t inline scale_q7( const q7_t v1, const q7_t v2, const q7_t dec2 )
{
    return (q7_t) __SSAT( (v1 * v2) >> dec2, 8 );
}

q7_t inline div_q7( const q7_t v1, const q7_t dec1, const q7_t v2, const q7_t dec2, const q7_t ret_dec )
{
    int8_t  lsh = ret_dec + dec2 - dec1;
    int32_t v = (int32_t)v1;

    if ( lsh > 0 ) {
        v = (v << lsh) / v2;
    } else if ( lsh < 0 ) {
        v = ( v / v2 ) >> (-lsh);
    } else
        v /= v2;

    return (q7_t) __SSAT( v, 8 );
}

q7_t inline ratio_q7( const q7_t v1, const q7_t v2, const q7_t ret_dec )
{
    return (q7_t) __SSAT( (((int32_t)v1) << ret_dec) / v2, 8 );
}

q15_t inline one_minus_q15( const q15_t v2, const q7_t dec )
{
    return (q15_t) __SSAT( (((int16_t)1) << dec) - v2, 16 );
}

q15_t inline scale_q15( const q15_t v1, const q15_t v2, const q7_t dec2 )
{
    return (q15_t) __SSAT( (v1 * v2) >> dec2, 16 );
}

q15_t inline ratio_q15( const q15_t v1, const q15_t v2, const q7_t ret_dec )
{
    return (q15_t) __SSAT( (((int32_t)v1) << ret_dec) / v2, 16 );
}

eta_rc DecodeCenterSizeBoxes( const CenterSizeEncoding * loc,
			                  const CenterSizeEncoding * anchors,
                              const int16_t * keep_indices,
                              const int16_t num_kept,
			                  BoxCornerEncodingFloat * bboxes,
			                  const ssd_detector_opt opt )
{
    // Parse input tensor boxencodings
    int16_t bbox_max   = ( 1 << opt.bbox_dec_bits ) - 1;
    float   bbox_quant_scale = (float) (bbox_max + 1);                      // float ==> Qm.n
    float   bbox_scale = qfp_fdiv( 1.0f, (float)bbox_quant_scale );         // Qm.n ==> float
    int8_t  oneOverScaleY = ratio_q7( 1, opt.scale_y, opt.bbox_dec_bits );
    int8_t  oneOverScaleX = ratio_q7( 1, opt.scale_x, opt.bbox_dec_bits );
    int8_t  oneOverScaleH = ratio_q7( 1, opt.scale_h, opt.bbox_dec_bits );
    int8_t  oneOverScaleW = ratio_q7( 1, opt.scale_w, opt.bbox_dec_bits );

    for ( int16_t ii = 0; ii < num_kept; ++ii ) {
        int16_t idx = keep_indices[ii];

        //float ycenter = loc.y / opt.scale_values.y * anchor.h + anchor.y;
        //float xcenter = loc.x / opt.scale_values.x * anchor.w + anchor.x;
        int8_t  ycenter = add_q7( scale_q7( loc[idx].y, scale_q7( oneOverScaleY, anchors[idx].h, opt.anchor_dec_bits ),
                                            opt.loc_dec_bits), opt.bbox_dec_bits,
                                  anchors[idx].y, opt.anchor_dec_bits, opt.bbox_dec_bits );
        int8_t  xcenter = add_q7( scale_q7( loc[idx].x, scale_q7( oneOverScaleX, anchors[idx].w, opt.anchor_dec_bits ),
                                            opt.loc_dec_bits), opt.bbox_dec_bits,
                                  anchors[idx].x, opt.anchor_dec_bits, opt.bbox_dec_bits );

        //float half_h = 0.5f * expf( loc.h / opt.scale_values.h ) * anchor.h;
        //float half_w = 0.5f * expf( loc.w / opt.scale_values.w ) * anchor.w;
        q7_t half_h = scale_q7( (q7_t)qfp_fmul( expf( qfp_fmul( (float)scale_q7( loc[idx].h, oneOverScaleH, opt.loc_dec_bits ), bbox_scale ) ),
                                                bbox_quant_scale ), anchors[idx].h, opt.anchor_dec_bits ) >> 1;
        q7_t half_w = scale_q7( (q7_t)qfp_fmul( expf( qfp_fmul( (float)scale_q7( loc[idx].w, oneOverScaleW, opt.loc_dec_bits ), bbox_scale ) ),
                                                bbox_quant_scale ), anchors[idx].w, opt.anchor_dec_bits ) >> 1;

        if ( half_h <= 0  ||  half_w <= 0 ) {
            return ETA_GEN_ERR;
        }

        bboxes[ii].ymin = max( ycenter - half_h, 0 );
        bboxes[ii].ymax = min( ycenter + half_h, bbox_max );
        bboxes[ii].xmin = max( xcenter - half_w, 0 );
        bboxes[ii].xmax = min( xcenter + half_w, bbox_max );
    }

    return ETA_STATUS_OK;
}

int16_t SelectDetectionsAboveScoreThresholdFloat( const q7_t * values, int16_t n,
                                            const q7_t threshold,
                                            q7_t * keep_values,
                                            int16_t * keep_indices )
{
    //printf( "\nSelectDetectionsAboveScoreThreshold\n" );

    // values[n, 2]: [nBoxes, bg or person]
    int16_t	count = 0;  // # of valid data
    int16_t jj = 1;     // index to person's conf-value

    for ( int16_t i = 0; i < n; i++, jj += 2 ) {
        //printf( "%4d: %3d %3d", i, values[jj - 1], values[jj] );
        if ( values[jj] >= threshold ) {
            keep_values[count] = values[jj];
            keep_indices[count] = i;
            count++;
            //printf( "  %4d", count );
        }
        //printf( "\n" );
    }

    return count;
}

int16_t SelectDetectionsAboveScoreThresholdConsideringBG( const q7_t * values, int16_t n,
                                                            const q7_t threshold,
                                                            q7_t * keep_values,
                                                            int16_t * keep_indices )
{
    //printf( "\nSelectDetectionsAboveScoreThresholdConsideringBG\n" );

    // values[n, 2]: [nBoxes, bg or person]
    int16_t	count = 0;  // # of valid data
    int16_t jj = 1;     // index to person's conf-value

    for ( int16_t i = 0; i < n; i++, jj += 2 ) {
        //printf( "%4d: %3d %3d", i, values[jj - 1], values[jj] );
        //if ( (values[jj - 1] >> 1) <= values[jj]                          // background conf * 4/8 <= person conf
        if ( (values[jj - 1] >> 2) + (values[jj - 1] >> 3) <= values[jj]    // background conf * 3/8 <= person conf
              &&  values[jj] >= threshold ) {
            keep_values[count] = values[jj];
            keep_indices[count] = i;
            count++;
            //printf( "  %4d", count );
        }
        //printf( "\n" );
    }

    return count;
}

/*
 * compute similarity between box_i and box_j by their aspect ratio and area ratio
 * input: bboxes in Q1.6 format
 * return: similarity in Qx.14
*/
int16_t similarity( const BoxCornerEncodingFloat box_i, const BoxCornerEncodingFloat box_j )
{
    int16_t wi_hj = ( box_i.xmax - box_i.xmin ) * ( box_j.ymax - box_j.ymin );
    int16_t wj_hi = ( box_j.xmax - box_j.xmin ) * ( box_i.ymax - box_i.ymin );
    int16_t aspect_ratio = wi_hj <= wj_hi ? ratio_q15( wi_hj, wj_hi, 14 ) : ratio_q15( wj_hi, wi_hj, 14 );      // Qx.14
    int16_t area_i = areaBox( box_i );
    int16_t area_j = areaBox( box_j );
    int16_t area_ratio = area_i <= area_j ? ratio_q15( area_i, area_j, 14 ) : ratio_q15( area_j, area_i, 14 );  // Qx.14

    return scale_q15( area_ratio, aspect_ratio, 14 );   // Qx.14
}


/*
 * compute Intersection-Over-Union between box_i and box_j
 * input: bboxes in Q1.6 format
 * return: intersection / union in Qx.14
*/
int16_t ComputeIntersectionOverUnion( const BoxCornerEncodingFloat box_i, const BoxCornerEncodingFloat box_j )
{
    const int16_t   intersection_ymin = max(box_i.ymin, box_j.ymin);
    const int16_t   intersection_xmin = max(box_i.xmin, box_j.xmin);
    const int16_t   intersection_ymax = min(box_i.ymax, box_j.ymax);
    const int16_t   intersection_xmax = min(box_i.xmax, box_j.xmax);
    const int16_t   area_intersection = max(intersection_ymax - intersection_ymin, 0 ) *
	                                    max(intersection_xmax - intersection_xmin, 0 );
    const int16_t   area_i = areaBox( box_i );
    const int16_t   area_j = areaBox( box_j );
    const int16_t   area_union = area_i + area_j - area_intersection;

    return area_intersection < area_union ? ratio_q15( area_intersection, area_union, 14 ) : (1 << 14);  // Qx.14
}

/*
int16_t ComputeUnionOverIntersection( const BoxCornerEncodingFloat* bboxes,
                                      const int16_t i, const int16_t j, const int16_t thresh )
{
    BoxCornerEncodingFloat   box_i = bboxes[i];
    BoxCornerEncodingFloat   box_j = bboxes[j];
    const int32_t   area_i = (box_i.ymax - box_i.ymin) * (box_i.xmax - box_i.xmin);
    const int32_t   area_j = (box_j.ymax - box_j.ymin) * (box_j.xmax - box_j.xmin);

    const int16_t intersection_ymin = max(box_i.ymin, box_j.ymin);
    const int16_t intersection_xmin = max(box_i.xmin, box_j.xmin);
    const int16_t intersection_ymax = min(box_i.ymax, box_j.ymax);
    const int16_t intersection_xmax = min(box_i.xmax, box_j.xmax);
    const int32_t intersection_area = max( intersection_ymax - intersection_ymin, 0 )
                                      * max( intersection_xmax - intersection_xmin, 0 );
#ifdef DEBUG
    printf( "box %4d: %7d (%6d %6d %6d %6d)\n", i, area_i, box_i.ymin, box_i.xmin, box_i.ymax, box_i.xmax );
    printf( "box %4d: %7d (%6d %6d %6d %6d)\n", j, area_j, box_j.ymin, box_j.xmin, box_j.ymax, box_j.xmax );
    printf( "Inter: %7d (%6d %6d %6d %6d)\n", intersection_area, intersection_ymin, intersection_xmin, intersection_ymax, intersection_xmax );
    printf( "Union: %7d\n", area_i + area_j - intersection_area );
#endif  // DEBUG

    return ((area_i + area_j - intersection_area) << kUoI_dec_bits) <= intersection_area * thresh;
}
*/

// NonMaxSuppressionSingleClass() prunes out the box locations with high overlap
// before selecting the highest scoring boxes (max_detections in number)
// It assumes all boxes are good in beginning and sorts based on the scores.
// If lower-scoring box has too much overlap with a higher-scoring box,
// we get rid of the lower-scoring box.
// Complexity is O(N^2) pairwise comparison between boxes
int16_t softNonMaxSuppressionSingleClassHelper( const BoxCornerEncodingFloat * bboxes,
                                            const q7_t * keep_scores,
                                            const int16_t num_kept,
                                            int16_t * selected_indices,
                                            const ssd_detector_opt opt )
{
    int16_t	*indices = selected_indices;
    int16_t scores[num_kept];               // Qx.14
    int16_t threshold_score = ((int16_t)opt.threshold_score) << (14 - opt.conf_dec_bits);  // Qx.conf_dec_bits ==> Qx.14

    // adjust confidence score based on the box size in favor to smaller box
    int8_t  lsh = 14 - ( opt.bbox_dec_bits * 2 );
    for ( int16_t ii = 0; ii < num_kept; ii++ ) {
#ifdef HARD_NMS
        int16_t scale = one_minus_q15( 0, 14 );                                 // Qx.14    hard-NMS
#else
        int16_t scale = one_minus_q15( areaBox( bboxes[ii] ) << lsh, 14 );      // Qx.14    soft-NMS
#endif  // HARD_NMS

        scores[ii] = ( scale * keep_scores[ii] ) >> opt.conf_dec_bits;      // Qx.14
        indices[ii] = ii;
    }
#if 0
    printf( "\r\n\nsoftNonMaxSuppressionSingleClassHelper: %d\n", num_kept );
    for ( int ii = 0; ii < num_kept; ii++ )
        printf( "  %4d=>%4d", keep_scores[ii], scores[ii] );
    printf( "\r\n\n" );
#endif  // DEBUG

    const int16_t   threshold_IOU = ((int16_t)opt.threshold_IOU) << (14 - 7);  // Q0.7 ==> Qx.14
    const int16_t	max_loop = min( opt.max_detections, num_kept );
    int16_t	count = 0;

    for ( int16_t i = 0; count < max_loop; ++i ) {
        // find the largest score
        int16_t maxV = threshold_score - 1;
        int16_t maxI = -1;

        for ( int16_t ii = i; ii < num_kept; ++ii )
            if ( maxV < scores[ii] ) {
                maxV = scores[ii];
                maxI = ii;
            }
        if ( maxI < 0 )
            break;

        int16_t idx = indices[maxI];

        scores[maxI] = scores[i];
        indices[maxI] = indices[i];
        indices[i] = idx;
        count++;

        // adjust confidence of the overlapped bboxes based on IOU & similarity
        const BoxCornerEncodingFloat bbox = bboxes[idx];

        for ( int16_t j = i + 1; j < num_kept; ++j ) {
            int16_t jj = indices[j];

            if ( scores[j] >= threshold_score ) {
                int16_t iou = ComputeIntersectionOverUnion( bbox, bboxes[jj] );  // Qx.14

                if ( iou >= threshold_IOU ) {	// overlapped
                    int16_t sm = similarity( bbox, bboxes[jj] );        // Qx.14
                    int16_t scale = scale_q15( one_minus_q15( iou, 14 ), one_minus_q15( sm, 14 ), 14 );    // Qx.14

#ifdef HARD_NMS
                    scores[j] = 0;                                      // hard-NMS
#else
                    scores[j] = scale_q15( scores[j], scale, 14 );      // soft-NMS
#endif  // HARD_NMS
                }
            }
        }
    }

    return count;
}


eta_rc eta_ssd_detector_q7( const q7_t * loc, 
                            const q7_t * conf, 
                            const q7_t * anchor,
                            q7_t * out_bboxes,
                            q7_t * out_class,
                            q7_t * out_conf,
                            int16_t * out_count,
                            const ssd_detector_opt opt )
{
    int16_t num_boxes = opt.num_boxes;
    // threshold confidence scores
    int16_t	keep_indices[num_boxes];
    q7_t	keep_scores[num_boxes];
    int16_t	num_kept;

    if ( opt.considering_BG == 0 )
        num_kept = SelectDetectionsAboveScoreThresholdFloat(
            conf, num_boxes, opt.threshold_score, keep_scores, keep_indices);
    else
    	num_kept = SelectDetectionsAboveScoreThresholdConsideringBG(
            conf, num_boxes, opt.threshold_score, keep_scores, keep_indices);

    // This fills in temporary decoded_boxes
    // by transforming input_box_encodings and input_anchors from
    // CenterSizeEncodings to BoxCornerEncodingFloat
    BoxCornerEncodingFloat	bboxes[num_kept];

    eta_rc  rc = DecodeCenterSizeBoxes( (CenterSizeEncoding *)loc,
                                        (CenterSizeEncoding *)anchor,
                                        keep_indices, num_kept,
                                        bboxes, opt );
    if ( rc == ETA_GEN_ERR ) {
        *out_count = 0;
        return rc;
    }

    // This fills in the output tensors
    // by choosing effective set of decoded boxes
    // based on Non Maximal Suppression, i.e. selecting
    // highest scoring non-overlapping boxes.
    int16_t	selected_indices[opt.max_detections];
    int16_t	nSelected = softNonMaxSuppressionSingleClassHelper( bboxes, keep_scores, num_kept, selected_indices, opt );
    BoxCornerEncodingFloat   *pout_box = (BoxCornerEncodingFloat *) out_bboxes;

    *out_count = nSelected;
    for ( int16_t ii = 0; ii < nSelected; ii++ ) {
        int16_t	jj = selected_indices[ii];

        out_conf[ii] = keep_scores[jj];
        out_class[ii] = 0;  // person
        pout_box[ii] = bboxes[jj];
        //printf( " (%d,%d)", out_conf[ii], keep_scores[jj] );
    }

    return ETA_STATUS_OK;
}


#ifdef __cplusplus
}
#endif
