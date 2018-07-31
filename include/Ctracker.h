#ifndef CTRACKER_H_
#define CTRACKER_H_

#include<Itracker.h>

namespace tracker
{
    class CTracker
    {
    public:
        CTracker(const Object &object,const size_t trackerID,const int frame_index); 
        void Update(const Object &object,const int frame_index);

        float CalcDistJaccard(const cv::Rect &r);
        Tracker tracker;
        int m_skippedFrames;
        int clsFrameNum;

    };

    CTracker::CTracker(const Object &object,const size_t trackerID,const int frame_index): m_skippedFrames(0),clsFrameNum(1)
    {
        tracker.id = trackerID;
        tracker.cls = object.second;
        tracker.track.push_back(object.first);
        tracker.start_frame = frame_index;
        tracker.end_frame = frame_index + 1;
    }

    float CTracker::CalcDistJaccard(const cv::Rect &r)
    {
        float intArea = (r & tracker.track.back()).area();
        float unionArea = (r | tracker.track.back()).area();

        return 1 - intArea/unionArea;
    }

    void CTracker::Update(const Object &object,const int frame_index)
    {
        tracker.track.push_back(object.first);
        if(object.second==tracker.cls)
        {
            clsFrameNum++;
        }
        else
        {
            if(clsFrameNum!=0)
            {
                clsFrameNum--;
            }
            else
            {
                tracker.cls = object.second;
                clsFrameNum++;
            }
        }

        tracker.end_frame = frame_index + 1;

    }

}
#endif
