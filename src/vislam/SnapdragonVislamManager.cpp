/****************************************************************************
 *   Copyright (c) 2016 Ramakrishna Kintada. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "SnapdragonVislamManager.hpp"
#include "SnapdragonDebugPrint.h"

Snapdragon::VislamManager::VislamManager(ros::NodeHandle nh) : nh_(nh) {
  cam_man_ptr_ = nullptr;
  vislam_ptr_ = nullptr;
  initialized_ = false;
  image_buffer_size_bytes_ = 0;
  image_buffer_ = nullptr;
}

Snapdragon::VislamManager::~VislamManager() {
  CleanUp();
}

void Snapdragon::VislamManager::ImuCallback(
    const sensor_msgs::Imu::ConstPtr& msg)
{
  int64_t current_timestamp_ns = msg->header.stamp.toNSec();

  float delta = 0.f;

  // Sanity check on IMU timestamp
  if (last_imu_timestamp_ns_ != 0)
  {
    delta = (current_timestamp_ns - last_imu_timestamp_ns_) * 1e-6;
    const float imu_sample_dt_reasonable_threshold_ms = 2.5;
    if (delta > imu_sample_dt_reasonable_threshold_ms)
    {
      if (cam_params_.verbose)
      {
        WARN_PRINT("IMU sample dt > %f ms -- %f ms",
                   imu_sample_dt_reasonable_threshold_ms, delta);
      }
    }
  }
  last_imu_timestamp_ns_ = current_timestamp_ns;

  ROS_INFO_STREAM_THROTTLE(1, "IMU timestamp [ns]: \t" << last_imu_timestamp_ns_);

  // Parse IMU message
  float lin_acc[3], ang_vel[3];

  // Convert ENU to NED coordinates
  lin_acc[0] = msg->linear_acceleration.x;
  lin_acc[1] = msg->linear_acceleration.y;
  lin_acc[2] = msg->linear_acceleration.z;
  ang_vel[0] = msg->angular_velocity.x;
  ang_vel[1] = msg->angular_velocity.y;
  ang_vel[2] = msg->angular_velocity.z;

  // Check for dropped IMU messages
  static uint32_t sequence_number_last = 0;
  int num_dropped_samples = 0;
  if (sequence_number_last != 0)
  {
    // The diff should be 1, anything greater means we dropped samples
    num_dropped_samples = msg->header.seq - sequence_number_last - 1;
    if (num_dropped_samples > 0)
    {
      if (cam_params_.verbose)
      {
        WARN_PRINT("Current IMU sample = %u, last IMU sample = %u",
                   msg->header.seq, sequence_number_last);
      }
    }
  }
  // sequence_number_last = imu_samples[ii].sequence_number;
  sequence_number_last = msg->header.seq;

  // Feed IMU message to VISLAM
  std::lock_guard<std::mutex> lock(sync_mutex_);
  mvVISLAM_AddAccel(vislam_ptr_, current_timestamp_ns, lin_acc[0], lin_acc[1],
                    lin_acc[2]);
  mvVISLAM_AddGyro(vislam_ptr_, current_timestamp_ns, ang_vel[0], ang_vel[1],
                   ang_vel[2]);
}

int32_t Snapdragon::VislamManager::CleanUp() {
  //stop the camera.
  if( cam_man_ptr_ != nullptr ) {
    WARN_PRINT( "Stopping Camera...." );
    cam_man_ptr_->Terminate();
    WARN_PRINT( "Deleting Camera Pointer" );
    delete cam_man_ptr_;
    cam_man_ptr_ = nullptr;
  }

  //stop the vislam engine.
  if( vislam_ptr_ != nullptr ) {
    mvVISLAM_Deinitialize( vislam_ptr_ );
    vislam_ptr_ = nullptr;
  }

  if( image_buffer_ != nullptr ) {
    delete[] image_buffer_;
    image_buffer_ = nullptr;
    image_buffer_size_bytes_ = 0;
  }
  return 0;
}

int32_t Snapdragon::VislamManager::Initialize
(
  const Snapdragon::CameraParameters& cam_params,
  const Snapdragon::VislamManager::InitParams& vislam_params
) {
  cam_params_ = cam_params;
  vislam_params_ = vislam_params;
  int32_t rc = 0;
  if( rc == 0 ) { //initialize the camera module.
    cam_man_ptr_ = new Snapdragon::CameraManager( &cam_params_ ) ;
    if( cam_man_ptr_ != nullptr ) {
      rc = cam_man_ptr_->Initialize();
    }
    else {
      rc = -1;
    }
  }

  //now intialize the VISLAM module.
  if( rc == 0 ) {
    vislam_ptr_ = mvVISLAM_Initialize
    (
      &(cam_params_.mv_camera_config), 0,
      vislam_params_.tbc, vislam_params_.ombc, vislam_params_.delta,
      vislam_params_.std0Tbc, vislam_params_.std0Ombc, vislam_params_.std0Delta,
      vislam_params_.accelMeasRange, vislam_params_.gyroMeasRange,
      vislam_params_.stdAccelMeasNoise, vislam_params_.stdGyroMeasNoise,
      vislam_params_.stdCamNoise, vislam_params_.minStdPixelNoise, vislam_params_.failHighPixelNoisePoints,
      vislam_params_.logDepthBootstrap, vislam_params_.useLogCameraHeight, vislam_params_.logCameraHeightBootstrap,
      vislam_params_.noInitWhenMoving,
      vislam_params_.limitedIMUbWtrigger
    );
    if( vislam_ptr_ == nullptr ) {
      rc = -1;
    }
  }

  if( rc != 0 ) {
    ERROR_PRINT( "Error initializing the Vislam Manager." );
    CleanUp();
  }
  else {
    initialized_ = true;
  }
  return 0;
}

int32_t Snapdragon::VislamManager::Start() {
  int32_t rc = 0;
  if( initialized_ ) {
    //start the camera
    rc |= cam_man_ptr_->Start();

    //wait till we get the first frame.
    while( cam_man_ptr_->GetLatestFrameId()  < 10 ) {
      std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
    }
    // allocate the image buffer here.
    image_buffer_size_bytes_ = cam_man_ptr_->GetImageSize();
    INFO_PRINT( "image Size: %d frameId: %lld", cam_man_ptr_->GetImageSize(), cam_man_ptr_->GetLatestFrameId() );
    image_buffer_ = new uint8_t[ image_buffer_size_bytes_ ];

    // Setup publishers/subscribers
    imu_sub_ = nh_.subscribe("mavros/imu/data_raw", 10,
                            &Snapdragon::VislamManager::ImuCallback, this);
  }
  else {
    ERROR_PRINT( "Calling Start without calling intialize" );
    rc = -1;
  }

  camera_update_thread_ = std::thread( &Snapdragon::VislamManager::getNextCameraImage, this);
  return rc;
}

int32_t Snapdragon::VislamManager::Stop() {
  CleanUp();

  // Unsubscribe from IMU topic
  imu_sub_.shutdown();

  if(camera_update_thread_.joinable()) {
    camera_update_thread_.join();
  }
  return 0;
}

int32_t Snapdragon::VislamManager::GetPointCloud( mvVISLAMMapPoint* points, uint32_t max_points ) {
  std::lock_guard<std::mutex> lock( sync_mutex_ );
  return mvVISLAM_GetPointCloud( vislam_ptr_, points, max_points );
}

bool Snapdragon::VislamManager::HasUpdatedPointCloud() {
  std::lock_guard<std::mutex> lock( sync_mutex_);
  int32_t mv_ret = mvVISLAM_HasUpdatedPointCloud( vislam_ptr_ );
  return (mv_ret != 0 );
}

int32_t Snapdragon::VislamManager::getNextCameraImage(){
  while(true){
    // ROS_INFO("getNextCameraImage() running");
    int32_t rc = 0;
    CameraImage camera_image;
    int64_t frame_id;
    static int64_t prev_frame_id = 0;
    uint32_t used = 0;
    uint64_t frame_ts_ns;

    rc = cam_man_ptr_->GetNextImageData( &frame_id, &frame_ts_ns, image_buffer_, image_buffer_size_bytes_ , &used );

    if( rc != 0 ) {
      WARN_PRINT( "Error Getting the image from camera" );
    }else{
      if( prev_frame_id != 0 && (prev_frame_id + 1 != frame_id ) ) {
        WARN_PRINT( "Warning: Missed/Dropped Camera Frames.  recvd(%lld) expected(%lld)", frame_id, (prev_frame_id+1) );
      }

      // adjust the frame-timestamp for VISLAM at it needs the time at the center of the exposure and not the sof.
      // Correction from exposure time
      float correction = 1e3 * (cam_man_ptr_->GetExposureTimeUs()/2.f);

      // Adjust timestamp with clock offset MONOTONIC <-> REALTIME:
      // Camera images will correctly have REALTIME timestamps, IMU messages
      // however carry the MONOTONIC timestamp.
      timespec mono_time, wall_time;
      if (clock_gettime(CLOCK_MONOTONIC, &mono_time) ||
          clock_gettime(CLOCK_REALTIME, &wall_time))
      {
        ERROR_PRINT("Cannot access clock time");
        continue;
      }
      // TODO: Perhaps filter clock_offset_ns. So far I only observed jumps of 1 milisecond though.
      int64_t clock_offset_ns = (wall_time.tv_sec - mono_time.tv_sec) * 1e9 +
                                 wall_time.tv_nsec - mono_time.tv_nsec;
      uint64_t modified_timestamp_ns = frame_ts_ns - static_cast<uint64_t>(correction) + clock_offset_ns;

      camera_image.frame_id = frame_id;
      camera_image.image_buffer = new uint8_t[image_buffer_size_bytes_];
      std::memcpy(camera_image.image_buffer, image_buffer_, image_buffer_size_bytes_);
      camera_image.buffer_size = image_buffer_size_bytes_;
      camera_image.frame_ts_ns = modified_timestamp_ns;

      std::lock_guard<std::mutex> buffer_lock(camera_buf_mutex_);
      camera_buffer_.push(camera_image);
    }
  }
  return 0;
}

int32_t Snapdragon::VislamManager::GetPose( mvVISLAMPose& pose, int64_t& pose_frame_id, uint64_t& timestamp_ns ) {
  ros::Time start_time = ros::Time::now();

  while(true){
    // Wait for data in camera buffer
    {
      std::lock_guard<std::mutex> buffer_lock(camera_buf_mutex_);
      if(!camera_buffer_.empty())
      {
        break;
      }
    }
    usleep(500);
  }

  while(true){
    {
      std::lock_guard<std::mutex> buffer_lock(camera_buf_mutex_);
      if (last_imu_timestamp_ns_>camera_buffer_.front().frame_ts_ns){
        break;
      }
    }
    usleep(500);
  }

  // ROS_INFO("Waited for %lld", (ros::Time::now()-start_time).toNSec());
  {
    std::lock_guard<std::mutex> lock( sync_mutex_ );
    std::lock_guard<std::mutex> buffer_lock(camera_buf_mutex_);
    mvVISLAM_AddImage(vislam_ptr_, camera_buffer_.front().frame_ts_ns, camera_buffer_.front().image_buffer);
    pose = mvVISLAM_GetPose(vislam_ptr_);
    pose_frame_id = camera_buffer_.front().frame_id;
    timestamp_ns = static_cast<uint64_t>(camera_buffer_.front().frame_ts_ns);
    ROS_INFO_STREAM_THROTTLE(1, "Image timestamp [ns]: \t" << timestamp_ns);
    camera_buffer_.pop();
  }
  return 0;
}
