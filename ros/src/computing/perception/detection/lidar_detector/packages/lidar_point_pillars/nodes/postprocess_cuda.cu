/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */



//headers in CUDA
#include <thrust/sort.h>

//headers in local files
#include "postprocess_cuda.h"

__global__ void filter_kernel(const float* box_preds, const float* cls_preds, const float* dir_preds, const int* anchor_mask,
                              const float* dev_anchors_px, const float* dev_anchors_py, const float* dev_anchors_pz,
                              const float* dev_anchors_dx, const float* dev_anchors_dy, const float* dev_anchors_dz, const float* dev_anchors_ro,
                              float* filtered_box, float* filtered_score, int* filtered_dir, float* box_for_nms, int* filter_count,
                              const float FLOAT_MIN, const float FLOAT_MAX, const float score_threshold)
{
  // boxes ([N, 7] Tensor): normal boxes: x, y, z, w, l, h, r
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  //sigmoid funciton
  float score = 1/(1+expf(-cls_preds[tid]));
  if(anchor_mask[tid] == 1 && score > score_threshold)
  {
    int counter = atomicAdd(filter_count, 1);
    float za = dev_anchors_pz[tid] + dev_anchors_dz[tid]/2;

    //decode network output
    float diagonal = sqrtf(dev_anchors_dx[tid]*dev_anchors_dx[tid] + dev_anchors_dy[tid]*dev_anchors_dy[tid]);
    float box_px = box_preds[tid*7 + 0] * diagonal + dev_anchors_px[tid];
    float box_py = box_preds[tid*7 + 1] * diagonal + dev_anchors_py[tid];
    float box_pz = box_preds[tid*7 + 2] * dev_anchors_dz[tid] + za;
    float box_dx = expf(box_preds[tid*7 + 3]) * dev_anchors_dx[tid];
    float box_dy = expf(box_preds[tid*7 + 4]) * dev_anchors_dy[tid];
    float box_dz = expf(box_preds[tid*7 + 5]) * dev_anchors_dz[tid];
    float box_ro = box_preds[tid*7 + 6] + dev_anchors_ro[tid];

    box_pz = box_pz - box_dz/2;

    filtered_box[counter*7 + 0] = box_px;
    filtered_box[counter*7 + 1] = box_py;
    filtered_box[counter*7 + 2] = box_pz;
    filtered_box[counter*7 + 3] = box_dx;
    filtered_box[counter*7 + 4] = box_dy;
    filtered_box[counter*7 + 5] = box_dz;
    filtered_box[counter*7 + 6] = box_ro;
    filtered_score[counter] = score;

    int direction_label;
    if(dir_preds[tid*2 + 0] < dir_preds[tid*2 + 1])
    {
      direction_label = 1;
    }
    else
    {
      direction_label = 0;
    }
    filtered_dir[counter] = direction_label;

    //convrt normal box(normal boxes: x, y, z, w, l, h, r) to box(xmin, ymin, xmax, ymax) for nms calculation
    //First: dx, dy -> box(x0y0, x0y1, x1y0, x1y1)
    float corners[2*4] = {float(-0.5*box_dx), float(-0.5*box_dy),
                          float(-0.5*box_dx), float( 0.5*box_dy),
                          float( 0.5*box_dx), float( 0.5*box_dy),
                          float( 0.5*box_dx), float(-0.5*box_dy)};

    //Second: Rotate, Offset and convert to point(xmin. ymin, xmax, ymax)
    float rotated_corners[2*4];
    float offset_corners[2*4];
    float sin_yaw = sinf(box_ro);
    float cos_yaw = cosf(box_ro);
    float xmin = FLOAT_MAX;
    float ymin = FLOAT_MAX;
    float xmax = FLOAT_MIN;
    float ymax = FLOAT_MIN;
    for(size_t i = 0; i < 4; i++)
    {
      rotated_corners[i*2 + 0] = cos_yaw*corners[i*2 + 0] - sin_yaw*corners[i*2 + 1];
      rotated_corners[i*2 + 1] = sin_yaw*corners[i*2 + 0] + cos_yaw*corners[i*2 + 1];

      offset_corners[i*2 + 0] = rotated_corners[i*2 + 0] + box_px;
      offset_corners[i*2 + 1] = rotated_corners[i*2 + 1] + box_py;

      xmin = fminf(xmin, offset_corners[i*2 + 0]);
      ymin = fminf(ymin, offset_corners[i*2 + 1]);
      xmax = fmaxf(xmin, offset_corners[i*2 + 0]);
      ymax = fmaxf(ymax, offset_corners[i*2 + 1]);
    }
    // box_for_nms(num_box, 4)
    box_for_nms[counter*4 + 0] = xmin;
    box_for_nms[counter*4 + 1] = ymin;
    box_for_nms[counter*4 + 2] = xmax;
    box_for_nms[counter*4 + 3] = ymax;

  }
}

__global__ void sort_boxes_by_indexes_kernel(float* filtered_box, int* filtered_dir, float* box_for_nms, int* indexes, int filter_count,
                                             float* sorted_filtered_boxes, int* sorted_filtered_dir, float* sorted_box_for_nms)
{
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  if(tid < filter_count)
  {
    int sort_index = indexes[tid];
    sorted_filtered_boxes[tid*7 + 0] = filtered_box[sort_index*7 + 0];
    sorted_filtered_boxes[tid*7 + 1] = filtered_box[sort_index*7 + 1];
    sorted_filtered_boxes[tid*7 + 2] = filtered_box[sort_index*7 + 2];
    sorted_filtered_boxes[tid*7 + 3] = filtered_box[sort_index*7 + 3];
    sorted_filtered_boxes[tid*7 + 4] = filtered_box[sort_index*7 + 4];
    sorted_filtered_boxes[tid*7 + 5] = filtered_box[sort_index*7 + 5];
    sorted_filtered_boxes[tid*7 + 6] = filtered_box[sort_index*7 + 6];

    sorted_filtered_dir[tid] = filtered_dir[sort_index];


    sorted_box_for_nms[tid*4 + 0] = box_for_nms[sort_index*4 + 0];
    sorted_box_for_nms[tid*4 + 1] = box_for_nms[sort_index*4 + 1];
    sorted_box_for_nms[tid*4 + 2] = box_for_nms[sort_index*4 + 2];
    sorted_box_for_nms[tid*4 + 3] = box_for_nms[sort_index*4 + 3];
  }
}

PostprocessCuda::PostprocessCuda(const float FLOAT_MIN, const float FLOAT_MAX,
  const int NUM_ANCHOR_X_INDS, const int NUM_ANCHOR_Y_INDS, const int NUM_ANCHOR_R_INDS,
  const float score_threshold, const int NUM_THREADS, const float nms_overlap_threshold):
FLOAT_MIN_(FLOAT_MIN),
FLOAT_MAX_(FLOAT_MAX),
NUM_ANCHOR_X_INDS_(NUM_ANCHOR_X_INDS),
NUM_ANCHOR_Y_INDS_(NUM_ANCHOR_Y_INDS),
NUM_ANCHOR_R_INDS_(NUM_ANCHOR_R_INDS),
score_threshold_(score_threshold),
NUM_THREADS_(NUM_THREADS),
nms_overlap_threshold_(nms_overlap_threshold)
{
  nms_cuda_ptr_.reset(new NMSCuda(
    NUM_THREADS,
    nms_overlap_threshold));
}

void PostprocessCuda::doPostprocessCuda(const float* rpn_box_output, const float* rpn_cls_output, const float* rpn_dir_output,
        int* dev_anchor_mask, const float* dev_anchors_px, const float* dev_anchors_py, const float* dev_anchors_pz,
        const float* dev_anchors_dx, const float* dev_anchors_dy, const float* dev_anchors_dz, const float* dev_anchors_ro,
        float* dev_filtered_box, float* dev_filtered_score, int* dev_filtered_dir, float* dev_box_for_nms, int* dev_filter_count,
        std::vector<float>& out_detection, int& out_num_objects)
{

  filter_kernel<<<NUM_ANCHOR_X_INDS_*NUM_ANCHOR_R_INDS_, NUM_ANCHOR_Y_INDS_>>>
                                (rpn_box_output, rpn_cls_output, rpn_dir_output, dev_anchor_mask,
                                dev_anchors_px, dev_anchors_py, dev_anchors_pz,
                                dev_anchors_dx, dev_anchors_dy, dev_anchors_dz, dev_anchors_ro,
                                dev_filtered_box, dev_filtered_score, dev_filtered_dir, dev_box_for_nms, dev_filter_count,
                                FLOAT_MIN_, FLOAT_MAX_, score_threshold_);

  int host_filter_count[1];
  GPU_CHECK( cudaMemcpy(host_filter_count, dev_filter_count, sizeof(int), cudaMemcpyDeviceToHost ) );
  if(host_filter_count[0] == 0)
  {
    return;
  }

  int* dev_indexes;
  float* dev_sorted_filtered_box,  *dev_sorted_box_for_nms;
  int* dev_sorted_filtered_dir;
  GPU_CHECK(cudaMalloc((void**)&dev_indexes, host_filter_count[0]*sizeof(int)));
  GPU_CHECK(cudaMalloc((void**)&dev_sorted_filtered_box, 7*host_filter_count[0]*sizeof(float)));
  GPU_CHECK(cudaMalloc((void**)&dev_sorted_filtered_dir, host_filter_count[0]*sizeof(int)));
  GPU_CHECK(cudaMalloc((void**)&dev_sorted_box_for_nms, 4*host_filter_count[0]*sizeof(float)));
  thrust::sequence(thrust::device, dev_indexes, dev_indexes + host_filter_count[0]);
  thrust::sort_by_key(thrust::device, dev_filtered_score, dev_filtered_score + size_t(host_filter_count[0]), dev_indexes, thrust::greater<float>());


  const int num_blocks = DIVUP(host_filter_count[0], NUM_THREADS_);
  sort_boxes_by_indexes_kernel<<<num_blocks, NUM_THREADS_>>>(dev_filtered_box, dev_filtered_dir, dev_box_for_nms, dev_indexes, host_filter_count[0],
                                                              dev_sorted_filtered_box, dev_sorted_filtered_dir, dev_sorted_box_for_nms);

  int keep_inds[host_filter_count[0]] = {0};
  nms_cuda_ptr_->doNMSCuda(host_filter_count[0], dev_sorted_box_for_nms, keep_inds, out_num_objects);


  float host_filtered_box[host_filter_count[0]*7];
  int host_filtered_dir[host_filter_count[0]];
  GPU_CHECK( cudaMemcpy(host_filtered_box, dev_sorted_filtered_box, 7*host_filter_count[0] *sizeof(float), cudaMemcpyDeviceToHost ) );
  GPU_CHECK( cudaMemcpy(host_filtered_dir, dev_sorted_filtered_dir,   host_filter_count[0] *sizeof(int), cudaMemcpyDeviceToHost ) );
  for (size_t i = 0; i < out_num_objects; i++)
  {
    out_detection.push_back(host_filtered_box[keep_inds[i]*7+0]);
    out_detection.push_back(host_filtered_box[keep_inds[i]*7+1]);
    out_detection.push_back(host_filtered_box[keep_inds[i]*7+2]);
    out_detection.push_back(host_filtered_box[keep_inds[i]*7+3]);
    out_detection.push_back(host_filtered_box[keep_inds[i]*7+4]);
    out_detection.push_back(host_filtered_box[keep_inds[i]*7+5]);

    if(host_filtered_dir[keep_inds[i]] == 0)
    {
      out_detection.push_back(host_filtered_box[keep_inds[i]*7+6] + M_PI);
    }
    else
    {
      out_detection.push_back(host_filtered_box[keep_inds[i]*7+6]);
    }
  }

  GPU_CHECK(cudaFree(dev_indexes));
  GPU_CHECK(cudaFree(dev_sorted_filtered_box));
  GPU_CHECK(cudaFree(dev_sorted_filtered_dir));
  GPU_CHECK(cudaFree(dev_sorted_box_for_nms));
}
