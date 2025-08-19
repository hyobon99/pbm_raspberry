#include "storage_manager.h"

namespace PhaseBasedMotionMagification {

int StorageManager::EnqueuePyramid(const Pyramid& newSlicedFourier) {   // reference
    std::unique_lock<std::mutex> lock(iMtx_Pyramid);
    ProcessMap_Pyramid.emplace(size_Pyramid, true);
    TimeSeries_Pyramid.emplace(size_Pyramid, newSlicedFourier);
    // TimeSeries_Pyramid.emplace_back(newSlicedFourier);
    std::cout << "(Pyramid) Enqueued, now size: " << size_Pyramid << std::endl;
    return size_Pyramid++;
}

// TODO: 특정 idx의 Pyramidal Phase는 looprange 갯수만큼 사용되어야지 모든 스레드가 사용 완료한 것임.
// 제거하는 조건을 이를 활용하여 제거하도록 변경
// 다만, LoopRange 값보다 이하의 초기 idx들은 해당 idx만큼의 사용횟수만 충족하면 됨
void StorageManager::DequeuePyramid(const int id) {
    std::cout << "(Pyramid) Current index : " << index_Pyramid << std::endl;
    std::unique_lock<std::mutex> lock(iMtx_Pyramid);
    
    // TimeSeries_Pyramid[id].clear();
    if (TimeSeries_Pyramid.find(id) == TimeSeries_Pyramid.end()) {
        std::cout << "(Pyramid) Invalid index: " << id << std::endl;
        return;
    } else {
        TimeSeries_Pyramid.erase(id);
        ProcessMap_Pyramid.erase(id);
        std::cout << "(Pyramid) Dequeued(hollowed), Idx: " << id << std::endl;
    }

    // for (int i = id + 1; i < size_Pyramid; i++) {
    //     if (ProcessMap_Pyramid[i]) {
    //         break;
    //     } else {
    //         // TimeSeries_Pyramid[i].clear();
    //         TimeSeries_Pyramid.erase(i);
    //         ProcessMap_Pyramid.erase(i);
    //         ++index_Pyramid;
    //         std::cout << "(Pyramid) Dequeued(hollowed), Idx: " << index_Pyramid << std::endl;
    //     }
    // }

    // if (id == index_Pyramid) {
    //     // TimeSeries_Pyramid[id].clear();
    //     TimeSeries_Pyramid.erase(id);
    //     ProcessMap_Pyramid.erase(id);
    //     std::cout << "(Pyramid) Dequeued(hollowed), Idx: " << index_Pyramid << std::endl;
    //     ++index_Pyramid;

    //     for (int i = id + 1; i < size_Pyramid; i++) {
    //         if (ProcessMap_Pyramid[i]) {
    //             break;
    //         } else {
    //             // TimeSeries_Pyramid[i].clear();
    //             TimeSeries_Pyramid.erase(i);
    //             ProcessMap_Pyramid.erase(i);
    //             ++index_Pyramid;
    //             std::cout << "(Pyramid) Dequeued(hollowed), Idx: " << index_Pyramid << std::endl;
    //         }
    //     }
    // } else {
    //     auto iter = ProcessMap_Pyramid.find(id);
    //     if (iter != ProcessMap_Pyramid.end()) iter->second = false;
    //     else std::cout << "(Pyramid) Invalid index: " << id << std::endl;

    //     std::cout << "(Pyramid) Dequeued(stacked), Idx: " << id << std::endl;
    // }
}

void StorageManager::LogPyramidUsage(const int id) {
    std::unique_lock<std::mutex> lock(iMtx_Pyramid_Usage);
    int targetId = id;

    if (UsageCount_Pyramid.find(targetId) == UsageCount_Pyramid.end()) {
        UsageCount_Pyramid.emplace(targetId, 1); 
    } else {
        UsageCount_Pyramid.at(targetId)++;
    }

    int thres_prev = (targetId + 1) * N_ORIENTATION * N_PYRAMID_LEVEL;
    int thres_main = N_LOOPRANGE * N_ORIENTATION * N_PYRAMID_LEVEL;

    if (targetId < N_LOOPRANGE) {
        // std::cout << "(Before Loop) Id: "<<targetId <<", Usage: " << UsageCount_Pyramid.at(targetId) << "/" << thres_prev << std::endl;
        if (UsageCount_Pyramid.at(targetId) == thres_prev) {
            DequeuePyramid(targetId);
            // std::cout << "(Pyramid) Usage count " << UsageCount_Pyramid.at(targetId) <<" for ID " << targetId << ", therefore deque." << std::endl;
        }
    } else {
        // std::cout << "(Main Loop) Id: "<< targetId << ", Usage: " << UsageCount_Pyramid.at(targetId) << "/" << thres_main << std::endl;
        if (UsageCount_Pyramid.at(targetId) == thres_main) {
            DequeuePyramid(targetId);
            // std::cout << "(Pyramid) Usage count " << UsageCount_Pyramid.at(targetId) <<" for ID " << targetId << ", therefore deque." << std::endl;
        }
    }

}

int StorageManager::EnqueueYiqFrame(const std::vector<cv::Mat>& newFrameYiq) {
    std::unique_lock<std::mutex> lock(iMtx_FrameYiq);
    ProcessMap_FrameYiq.emplace(size_FrameYiq, true);
    Map_FrameYiq.emplace(size_FrameYiq, newFrameYiq);
    std::cout << "(FrameYiq) Enqueued, now size: " << size_FrameYiq << std::endl;
    return size_FrameYiq++;
}

void StorageManager::DequeueYiqFrame(const int id) {
    std::unique_lock<std::mutex> lock(iMtx_FrameYiq);
    std::cout << "(FrameYiq) Current index : " << index_FrameYiq << std::endl;
    if (id == index_FrameYiq) {
        Map_FrameYiq.erase(id);
        ProcessMap_FrameYiq.erase(id);
        std::cout << "(FrameYiq) Dequeued(hollowed), Idx: " << index_FrameYiq << std::endl;
        ++index_FrameYiq;

        for (int i = id + 1; i < size_FrameYiq; i++) {
            if (ProcessMap_FrameYiq[i]) {
                break;
            } else {
                Map_FrameYiq.erase(id);
                ProcessMap_FrameYiq.erase(i);
                ++index_FrameYiq;
                std::cout << "(FrameYiq) Dequeued(hollowed), Idx: " << index_FrameYiq << std::endl;
            }
        }
    } else {
        auto iter = ProcessMap_FrameYiq.find(id);
        if (iter != ProcessMap_FrameYiq.end()) iter->second = false;
        else std::cout << "(FrameYiq) Invalid index: " << id << std::endl;

        std::cout << "(FrameYiq) Dequeued(stacked), Idx: " << id << std::endl;
    }
}

};