#ifndef _ROS_rtabmap_ros_OdomInfo_h
#define _ROS_rtabmap_ros_OdomInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "rtabmap_ros/KeyPoint.h"
#include "rtabmap_ros/Point2f.h"
#include "geometry_msgs/Transform.h"

namespace rtabmap_ros
{

  class OdomInfo : public ros::Msg
  {
    public:
      std_msgs::Header header;
      bool lost;
      int32_t matches;
      int32_t inliers;
      float variance;
      int32_t features;
      int32_t localMapSize;
      float timeEstimation;
      float timeParticleFiltering;
      float stamp;
      float interval;
      float distanceTravelled;
      int32_t type;
      uint8_t wordsKeys_length;
      int32_t st_wordsKeys;
      int32_t * wordsKeys;
      uint8_t wordsValues_length;
      rtabmap_ros::KeyPoint st_wordsValues;
      rtabmap_ros::KeyPoint * wordsValues;
      uint8_t wordMatches_length;
      int32_t st_wordMatches;
      int32_t * wordMatches;
      uint8_t wordInliers_length;
      int32_t st_wordInliers;
      int32_t * wordInliers;
      uint8_t refCorners_length;
      rtabmap_ros::Point2f st_refCorners;
      rtabmap_ros::Point2f * refCorners;
      uint8_t newCorners_length;
      rtabmap_ros::Point2f st_newCorners;
      rtabmap_ros::Point2f * newCorners;
      uint8_t cornerInliers_length;
      int32_t st_cornerInliers;
      int32_t * cornerInliers;
      geometry_msgs::Transform transform;
      geometry_msgs::Transform transformFiltered;

    OdomInfo():
      header(),
      lost(0),
      matches(0),
      inliers(0),
      variance(0),
      features(0),
      localMapSize(0),
      timeEstimation(0),
      timeParticleFiltering(0),
      stamp(0),
      interval(0),
      distanceTravelled(0),
      type(0),
      wordsKeys_length(0), wordsKeys(NULL),
      wordsValues_length(0), wordsValues(NULL),
      wordMatches_length(0), wordMatches(NULL),
      wordInliers_length(0), wordInliers(NULL),
      refCorners_length(0), refCorners(NULL),
      newCorners_length(0), newCorners(NULL),
      cornerInliers_length(0), cornerInliers(NULL),
      transform(),
      transformFiltered()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_lost;
      u_lost.real = this->lost;
      *(outbuffer + offset + 0) = (u_lost.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lost);
      union {
        int32_t real;
        uint32_t base;
      } u_matches;
      u_matches.real = this->matches;
      *(outbuffer + offset + 0) = (u_matches.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_matches.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_matches.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_matches.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->matches);
      union {
        int32_t real;
        uint32_t base;
      } u_inliers;
      u_inliers.real = this->inliers;
      *(outbuffer + offset + 0) = (u_inliers.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_inliers.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_inliers.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_inliers.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->inliers);
      union {
        float real;
        uint32_t base;
      } u_variance;
      u_variance.real = this->variance;
      *(outbuffer + offset + 0) = (u_variance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_variance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_variance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_variance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->variance);
      union {
        int32_t real;
        uint32_t base;
      } u_features;
      u_features.real = this->features;
      *(outbuffer + offset + 0) = (u_features.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_features.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_features.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_features.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->features);
      union {
        int32_t real;
        uint32_t base;
      } u_localMapSize;
      u_localMapSize.real = this->localMapSize;
      *(outbuffer + offset + 0) = (u_localMapSize.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_localMapSize.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_localMapSize.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_localMapSize.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->localMapSize);
      union {
        float real;
        uint32_t base;
      } u_timeEstimation;
      u_timeEstimation.real = this->timeEstimation;
      *(outbuffer + offset + 0) = (u_timeEstimation.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timeEstimation.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_timeEstimation.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_timeEstimation.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeEstimation);
      union {
        float real;
        uint32_t base;
      } u_timeParticleFiltering;
      u_timeParticleFiltering.real = this->timeParticleFiltering;
      *(outbuffer + offset + 0) = (u_timeParticleFiltering.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timeParticleFiltering.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_timeParticleFiltering.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_timeParticleFiltering.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeParticleFiltering);
      union {
        float real;
        uint32_t base;
      } u_stamp;
      u_stamp.real = this->stamp;
      *(outbuffer + offset + 0) = (u_stamp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stamp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stamp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stamp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp);
      union {
        float real;
        uint32_t base;
      } u_interval;
      u_interval.real = this->interval;
      *(outbuffer + offset + 0) = (u_interval.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_interval.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_interval.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_interval.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->interval);
      union {
        float real;
        uint32_t base;
      } u_distanceTravelled;
      u_distanceTravelled.real = this->distanceTravelled;
      *(outbuffer + offset + 0) = (u_distanceTravelled.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distanceTravelled.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distanceTravelled.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distanceTravelled.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distanceTravelled);
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_type.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset++) = wordsKeys_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < wordsKeys_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_wordsKeysi;
      u_wordsKeysi.real = this->wordsKeys[i];
      *(outbuffer + offset + 0) = (u_wordsKeysi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wordsKeysi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wordsKeysi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wordsKeysi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wordsKeys[i]);
      }
      *(outbuffer + offset++) = wordsValues_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < wordsValues_length; i++){
      offset += this->wordsValues[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = wordMatches_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < wordMatches_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_wordMatchesi;
      u_wordMatchesi.real = this->wordMatches[i];
      *(outbuffer + offset + 0) = (u_wordMatchesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wordMatchesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wordMatchesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wordMatchesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wordMatches[i]);
      }
      *(outbuffer + offset++) = wordInliers_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < wordInliers_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_wordInliersi;
      u_wordInliersi.real = this->wordInliers[i];
      *(outbuffer + offset + 0) = (u_wordInliersi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wordInliersi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wordInliersi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wordInliersi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wordInliers[i]);
      }
      *(outbuffer + offset++) = refCorners_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < refCorners_length; i++){
      offset += this->refCorners[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = newCorners_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < newCorners_length; i++){
      offset += this->newCorners[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = cornerInliers_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < cornerInliers_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_cornerInliersi;
      u_cornerInliersi.real = this->cornerInliers[i];
      *(outbuffer + offset + 0) = (u_cornerInliersi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cornerInliersi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cornerInliersi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cornerInliersi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cornerInliers[i]);
      }
      offset += this->transform.serialize(outbuffer + offset);
      offset += this->transformFiltered.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_lost;
      u_lost.base = 0;
      u_lost.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->lost = u_lost.real;
      offset += sizeof(this->lost);
      union {
        int32_t real;
        uint32_t base;
      } u_matches;
      u_matches.base = 0;
      u_matches.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_matches.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_matches.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_matches.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->matches = u_matches.real;
      offset += sizeof(this->matches);
      union {
        int32_t real;
        uint32_t base;
      } u_inliers;
      u_inliers.base = 0;
      u_inliers.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_inliers.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_inliers.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_inliers.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->inliers = u_inliers.real;
      offset += sizeof(this->inliers);
      union {
        float real;
        uint32_t base;
      } u_variance;
      u_variance.base = 0;
      u_variance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_variance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_variance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_variance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->variance = u_variance.real;
      offset += sizeof(this->variance);
      union {
        int32_t real;
        uint32_t base;
      } u_features;
      u_features.base = 0;
      u_features.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_features.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_features.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_features.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->features = u_features.real;
      offset += sizeof(this->features);
      union {
        int32_t real;
        uint32_t base;
      } u_localMapSize;
      u_localMapSize.base = 0;
      u_localMapSize.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_localMapSize.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_localMapSize.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_localMapSize.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->localMapSize = u_localMapSize.real;
      offset += sizeof(this->localMapSize);
      union {
        float real;
        uint32_t base;
      } u_timeEstimation;
      u_timeEstimation.base = 0;
      u_timeEstimation.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_timeEstimation.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_timeEstimation.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_timeEstimation.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->timeEstimation = u_timeEstimation.real;
      offset += sizeof(this->timeEstimation);
      union {
        float real;
        uint32_t base;
      } u_timeParticleFiltering;
      u_timeParticleFiltering.base = 0;
      u_timeParticleFiltering.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_timeParticleFiltering.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_timeParticleFiltering.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_timeParticleFiltering.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->timeParticleFiltering = u_timeParticleFiltering.real;
      offset += sizeof(this->timeParticleFiltering);
      union {
        float real;
        uint32_t base;
      } u_stamp;
      u_stamp.base = 0;
      u_stamp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stamp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stamp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stamp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stamp = u_stamp.real;
      offset += sizeof(this->stamp);
      union {
        float real;
        uint32_t base;
      } u_interval;
      u_interval.base = 0;
      u_interval.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_interval.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_interval.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_interval.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->interval = u_interval.real;
      offset += sizeof(this->interval);
      union {
        float real;
        uint32_t base;
      } u_distanceTravelled;
      u_distanceTravelled.base = 0;
      u_distanceTravelled.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distanceTravelled.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distanceTravelled.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distanceTravelled.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distanceTravelled = u_distanceTravelled.real;
      offset += sizeof(this->distanceTravelled);
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->type = u_type.real;
      offset += sizeof(this->type);
      uint8_t wordsKeys_lengthT = *(inbuffer + offset++);
      if(wordsKeys_lengthT > wordsKeys_length)
        this->wordsKeys = (int32_t*)realloc(this->wordsKeys, wordsKeys_lengthT * sizeof(int32_t));
      offset += 3;
      wordsKeys_length = wordsKeys_lengthT;
      for( uint8_t i = 0; i < wordsKeys_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_wordsKeys;
      u_st_wordsKeys.base = 0;
      u_st_wordsKeys.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_wordsKeys.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_wordsKeys.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_wordsKeys.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_wordsKeys = u_st_wordsKeys.real;
      offset += sizeof(this->st_wordsKeys);
        memcpy( &(this->wordsKeys[i]), &(this->st_wordsKeys), sizeof(int32_t));
      }
      uint8_t wordsValues_lengthT = *(inbuffer + offset++);
      if(wordsValues_lengthT > wordsValues_length)
        this->wordsValues = (rtabmap_ros::KeyPoint*)realloc(this->wordsValues, wordsValues_lengthT * sizeof(rtabmap_ros::KeyPoint));
      offset += 3;
      wordsValues_length = wordsValues_lengthT;
      for( uint8_t i = 0; i < wordsValues_length; i++){
      offset += this->st_wordsValues.deserialize(inbuffer + offset);
        memcpy( &(this->wordsValues[i]), &(this->st_wordsValues), sizeof(rtabmap_ros::KeyPoint));
      }
      uint8_t wordMatches_lengthT = *(inbuffer + offset++);
      if(wordMatches_lengthT > wordMatches_length)
        this->wordMatches = (int32_t*)realloc(this->wordMatches, wordMatches_lengthT * sizeof(int32_t));
      offset += 3;
      wordMatches_length = wordMatches_lengthT;
      for( uint8_t i = 0; i < wordMatches_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_wordMatches;
      u_st_wordMatches.base = 0;
      u_st_wordMatches.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_wordMatches.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_wordMatches.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_wordMatches.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_wordMatches = u_st_wordMatches.real;
      offset += sizeof(this->st_wordMatches);
        memcpy( &(this->wordMatches[i]), &(this->st_wordMatches), sizeof(int32_t));
      }
      uint8_t wordInliers_lengthT = *(inbuffer + offset++);
      if(wordInliers_lengthT > wordInliers_length)
        this->wordInliers = (int32_t*)realloc(this->wordInliers, wordInliers_lengthT * sizeof(int32_t));
      offset += 3;
      wordInliers_length = wordInliers_lengthT;
      for( uint8_t i = 0; i < wordInliers_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_wordInliers;
      u_st_wordInliers.base = 0;
      u_st_wordInliers.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_wordInliers.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_wordInliers.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_wordInliers.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_wordInliers = u_st_wordInliers.real;
      offset += sizeof(this->st_wordInliers);
        memcpy( &(this->wordInliers[i]), &(this->st_wordInliers), sizeof(int32_t));
      }
      uint8_t refCorners_lengthT = *(inbuffer + offset++);
      if(refCorners_lengthT > refCorners_length)
        this->refCorners = (rtabmap_ros::Point2f*)realloc(this->refCorners, refCorners_lengthT * sizeof(rtabmap_ros::Point2f));
      offset += 3;
      refCorners_length = refCorners_lengthT;
      for( uint8_t i = 0; i < refCorners_length; i++){
      offset += this->st_refCorners.deserialize(inbuffer + offset);
        memcpy( &(this->refCorners[i]), &(this->st_refCorners), sizeof(rtabmap_ros::Point2f));
      }
      uint8_t newCorners_lengthT = *(inbuffer + offset++);
      if(newCorners_lengthT > newCorners_length)
        this->newCorners = (rtabmap_ros::Point2f*)realloc(this->newCorners, newCorners_lengthT * sizeof(rtabmap_ros::Point2f));
      offset += 3;
      newCorners_length = newCorners_lengthT;
      for( uint8_t i = 0; i < newCorners_length; i++){
      offset += this->st_newCorners.deserialize(inbuffer + offset);
        memcpy( &(this->newCorners[i]), &(this->st_newCorners), sizeof(rtabmap_ros::Point2f));
      }
      uint8_t cornerInliers_lengthT = *(inbuffer + offset++);
      if(cornerInliers_lengthT > cornerInliers_length)
        this->cornerInliers = (int32_t*)realloc(this->cornerInliers, cornerInliers_lengthT * sizeof(int32_t));
      offset += 3;
      cornerInliers_length = cornerInliers_lengthT;
      for( uint8_t i = 0; i < cornerInliers_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_cornerInliers;
      u_st_cornerInliers.base = 0;
      u_st_cornerInliers.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cornerInliers.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cornerInliers.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cornerInliers.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cornerInliers = u_st_cornerInliers.real;
      offset += sizeof(this->st_cornerInliers);
        memcpy( &(this->cornerInliers[i]), &(this->st_cornerInliers), sizeof(int32_t));
      }
      offset += this->transform.deserialize(inbuffer + offset);
      offset += this->transformFiltered.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/OdomInfo"; };
    const char * getMD5(){ return "bb11dda76811d80a5965a15c6b7baf24"; };

  };

}
#endif