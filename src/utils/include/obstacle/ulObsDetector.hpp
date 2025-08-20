#ifndef ULOBS_DETECTOR_HPP_
#define ULOBS_DETECTOR_HPP_


#include <string.h>

#define SHANGHAI 0

typedef enum obsDir
{
    #if SHANGHAI
        FRONT0 = 2 ,
        FRONT1 = 3,
        RIGHT0 = 6,
        RIGHT1 = 7,
        BACK0 = 4,
        BACK1 = 5,
        LEFT0 = 0,
        LEFT1 = 1
    #else
        FRONT0 = 0 ,
        FRONT1 = 1,
        RIGHT0 = 2,
        RIGHT1 = 3,
        BACK0 = 4,
        BACK1 = 5,
        LEFT0 = 6,
        LEFT1 = 7
    #endif
};

/*
字段说明‌
    ‌flObs (Front Left Obstacle)‌
        类型：bool（布尔值）
        含义：左侧前方超声波传感器是否检测到障碍物（true=有障碍，false=无障碍）。
    ‌frObs (Front Right Obstacle)‌
        类型：bool
        含义：右侧前方超声波传感器是否检测到障碍物。
    ‌rObs (Right Obstacle)‌
        类型：bool
        含义：右侧传感器是否检测到障碍物。
    ‌lObs (Left Obstacle)‌
        类型：bool
        含义：左侧传感器是否检测到障碍物。
    ‌bObs (Back Obstacle)‌
        类型：bool
        含义：后方传感器是否检测到障碍物。
*/
struct UltrasonicObstacleState {
    bool flObs;   
    bool frObs;
    bool rObs;   
    bool lObs;    
    bool bObs;   
};

class UltrasonicObstacleDetector {
    public:
        void updateData(const unsigned char* newData) {
            memcpy(ulbr[cbid_], newData, sizeof(unsigned char) * 8);
            cbid_ = (cbid_ + 1) % ubs;
    
            checkObstacleState();
        }
    
        UltrasonicObstacleState getObstacleState() const {
            return obss_;
        }
        unsigned char fl = 0,fr = 0;
        unsigned char  getFrontLeft()
        {
            return  fl;
        }

        unsigned char  getFrontRight()
        {
            return  fr;
        }
        
    private:
        static const int ubs = 5; 
        unsigned char ulbr[ubs][8];
        int cbid_ = 0;
        UltrasonicObstacleState obss_;
    
        void checkObstacleState() {
            int flct = 0;
            int frct = 0;
            int bct = 0;
            int rct = 0;
            int lct = 0;
    
            for (int i = 0; i < ubs; ++i) {
                if (iflobs(ulbr[i]))flct++;
                if (ifrobs(ulbr[i]))frct++;
                if (ibobs(ulbr[i])) bct++;
                if (irobs(ulbr[i])) rct++;
                if (ilobs(ulbr[i])) lct++;
            }
    
            obss_.flObs = flct >= ubs - 1;
            obss_.frObs = frct >= ubs - 1;
            obss_.bObs = bct >= ubs - 1;
            obss_.rObs = (rct >= ubs - 1);
            obss_.lObs = (lct >= ubs - 1);
        }
    
        unsigned char iflobs(const unsigned char* data) {
            if (data[FRONT0] != 1 && data[FRONT0] != 255 && data[FRONT0] < 70)
            {
                fl = data[FRONT0];
                return  data[FRONT0];
            }
            else
                return 0;
        }

        unsigned char ifrobs(const unsigned char* data) {          
            if (data[FRONT1] != 1 && data[FRONT1] != 255 && data[FRONT1] < 70) 
            {
                fr = data[FRONT1];
                return  data[FRONT1];
            }              
            else
                return 0;
        }
    
        bool ibobs(const unsigned char* data) {
            if (data[BACK0] != 1 && data[BACK0] != 255 && 20 < data[BACK0] && data[BACK0] < 80) return true;
            if (data[BACK1] != 1 && data[BACK1] != 255 && 20 < data[BACK1] && data[BACK1] < 80) return true;
            return false;
        }
    
        bool irobs(const unsigned char* data) {
            if (data[RIGHT0] != 1 && data[RIGHT0] != 255 && 20 < data[RIGHT0] && data[RIGHT0] < 70) return true;
            if (data[RIGHT1] != 1 && data[RIGHT1] != 255 && 20 < data[RIGHT1] && data[RIGHT1] < 70) return true;
            return false;
        }
    
        bool ilobs(const unsigned char* data) {
            if (data[LEFT0] != 1 && data[LEFT0] != 255 && 20 < data[LEFT0] && data[LEFT0] < 70) return true;
            if (data[LEFT1] != 1 && data[LEFT1] != 255 && 20 < data[LEFT1] && data[LEFT1] < 70) return true;
            return false;
        }
    };


#endif
