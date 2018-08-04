#include<Itracker.h>
#include<Ctracker.h>
#include<HungarianAlg.h>
#include<util.h>
#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;

namespace tracker
{
    class Trackers : public ITrackers
    {
    public:
        Trackers(float dist_thres,int maximum_allowed_skipped_frames,float mergeThreshold,int output_min_tracker_length);
        virtual void Update(const vector<Object> objects,int frame_index);
        virtual void getTracks(vector<Tracker> &tr_over,vector<Tracker> &tr_running);

    private:
        vector<std::unique_ptr<CTracker> > trackers_running;
        vector<std::unique_ptr<CTracker> > trackers_over;
        size_t nextTrackID;
        float dist_thres;
        int maximum_allowed_skipped_frames;
        float mergeThreshold;
        int output_min_tracker_length;
    };

    #define GetRectCenter(rect) Point(rect.x+rect.width/2,rect.y+rect.height/2)

    Trackers::Trackers(float dist_thres,int maximum_allowed_skipped_frames,float mergeThreshold,int output_min_tracker_length):nextTrackID(0),
        dist_thres(dist_thres),
        maximum_allowed_skipped_frames(maximum_allowed_skipped_frames),
        mergeThreshold(mergeThreshold),
        output_min_tracker_length(output_min_tracker_length)
    {
    }

    float CalcDistIOU(const cv::Rect& r1,const cv::Rect& r2)
    {
        float intArea = (r1 & r2).area();
        float unionArea = (r1 | r2).area();

        return intArea/unionArea;
    }

    void Trackers::Update(const vector<Object> objects,int frame_index)
    {
        if(!trackers_running.size())
        {
            for(int i=0; i<objects.size(); ++i)
            {
                trackers_running.push_back(std::make_unique<CTracker>(objects[i],nextTrackID++,frame_index));
            }
            return;
        }

        if(objects.empty())
        {
            for(int i=0; i<trackers_running.size(); ++i)
            {
                trackers_running[i]->m_skippedFrames++;
            }

            return;
        }

        int N = trackers_running.size();
        int M = objects.size();

        vector<int> assignment(N,-1);
        vector<float> Cost(N*M);

        //cost matrix
        for(int i=0; i<M; ++i)
        {
            int indexTemp = i*N;
            for(int j=0; j<N; ++j)
            {
                Cost[indexTemp+j] = trackers_running[j]->CalcDistJaccard(objects[i].first);
            }
        }

        //match
        AssignmentProblemSolver APS;
        APS.Solve(Cost,N,M,assignment,AssignmentProblemSolver::optimal);

        //Update old tracker 
        for(int i=0; i<assignment.size(); ++i)
        {
            if(assignment[i]==-1)
            {
                trackers_running[i]->m_skippedFrames++;
            }
            else if(Cost[i+assignment[i]*N]>=dist_thres)
            {
                assignment[i] = -1;
                trackers_running[i]->m_skippedFrames++;
            }
            else
            {
                Object obj;
                obj.second = trackers_running[i]->tracker.cls;
                obj.first = trackers_running[i]->tracker.track.back();

                //int start_frame = trackers_running[i]->tracker.start_frame;
                //int end_frame = trackers_running[i]->tracker.end_frame;
                //int track_size = trackers_running[i]->tracker.track.size();
                //cout<<"befor update: "<<start_frame<<"\t"<<end_frame<<"\t"<<track_size<<endl;

                for(int j=0; j<trackers_running[i]->m_skippedFrames; ++j)
                {
                    trackers_running[i]->Update(obj,frame_index);
                }

                trackers_running[i]->m_skippedFrames = 0;

                trackers_running[i]->Update(objects[assignment[i]],frame_index);
            }

        }

        //new tracker
        for(int i=0; i<objects.size(); ++i)
        {
            if(find(assignment.begin(), assignment.end(), i)==assignment.end())
            {
                trackers_running.push_back(std::make_unique<CTracker>(objects[i],nextTrackID++,frame_index));
            }
        }

        for(int i=0; i<trackers_running.size(); ++i)
        {
            if(trackers_running[i]->m_skippedFrames>maximum_allowed_skipped_frames)
            {

                trackers_over.push_back(std::move(trackers_running[i]));
                trackers_running.erase(trackers_running.begin()+i);
                --i;
            }
        }

        //merge over IOU threhold tracker
        while(1)
        {
            int N = trackers_running.size();
            float iou = 0;
            int IOUPart[2] = {0,0};

            for(int i=0; i<N; ++i)
            {
                for(int j = i+1; j<N; ++j)
                {
                    float temp = CalcDistIOU(trackers_running[i]->tracker.track.back(),trackers_running[j]->tracker.track.back());
                    if(iou<temp)
                    {
                        iou = temp;
                        IOUPart[0] = i;
                        IOUPart[1] = j;
                    }

                }
            }

            if(iou>mergeThreshold)
            {
                int deleteTrackIdx = trackers_running[IOUPart[0]]->tracker.track.size()>trackers_running[IOUPart[1]]->tracker.track.size() ? IOUPart[1] : IOUPart[0];

                if(deleteTrackIdx==IOUPart[0])
                    trackers_running[IOUPart[1]]->m_skippedFrames = 0;
                else
                    trackers_running[IOUPart[0]]->m_skippedFrames = 0;

                trackers_running.erase(trackers_running.begin()+deleteTrackIdx);
            }
            else
                break;
        }
    }

    void Trackers::getTracks(vector<Tracker> &tr_over,vector<Tracker> &tr_running)
    {
        for(int i=0; i<trackers_running.size(); ++i)
        {
            if(trackers_running[i]->tracker.track.size()<output_min_tracker_length)
                continue;

            tr_running.push_back(trackers_running[i]->tracker);
        }

        for(int i=0; i<trackers_over.size(); ++i)
        {
            tr_over.push_back(trackers_over[i]->tracker);
        }
    }

    ITrackers *CreateITrackers()
    { 
        return new Trackers(1,5,0.7,10);
    }
}
