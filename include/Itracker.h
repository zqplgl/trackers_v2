#ifndef ITRACKER_H_
#define ITRACKER_H_

#include<opencv2/opencv.hpp>
#include<vector>
#include<utility>
#include <string>
using namespace std;

namespace tracker
{
    typedef pair<cv::Rect,int> Object;
    typedef vector<cv::Rect> Track;

    struct Tracker
    {
        Track track;
        size_t id;
        int start_frame;
        int end_frame;
        int cls;
    };

    class ITrackers
    {
    public:
        virtual void Update(const vector<Object> objects,int frame_index)=0;
        virtual void getTracks(vector<Tracker> &trackers_over,vector<Tracker> &trackers_running)=0;
        virtual ~ITrackers(){}
    };

    ITrackers *CreateITrackers();
}

#endif

