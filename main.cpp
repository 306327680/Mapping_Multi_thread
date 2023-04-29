#include <iostream>
#include "include/mapping.h"
#include <pthread.h>
#include<math.h>
#include <mutex>
#define NUM_THREADS 18
typedef struct{
	pthread_mutex_t lok;
    int tid = 0;
    std::vector<std::vector<std::string>> file_names_all;
}str_st;
str_st tmp;

typedef struct MAPPING_PARAMS {
    int tid = 0;
    std::vector<std::string> file_names_all;
} MappingParams;

std::vector<MappingParams> ParamVector;

bool GetFileNames(std::vector<std::string> &file_names_, const std::string directory, const std::string suffix="pcd"){
    file_names_.clear();
    DIR *dp;
    struct dirent *dirp;
    dp = opendir(directory.c_str());
    if (!dp) {
        std::cerr << "cannot open directory:" << directory << std::endl;
        return false;
    }
    std::string file;
    while (dirp = readdir(dp)) {
        file = dirp->d_name;
        if (file.find(".") != std::string::npos) {
            file = directory + "/" + file;
            if (suffix == file.substr(file.size() - suffix.size())) {
                file_names_.push_back(file);
            }
        }
    }
    closedir(dp);
    std::sort(file_names_.begin(), file_names_.end());

    if (file_names_.empty()) {
        std::cerr << "directory:" << directory << "is empty" << std::endl;
        return false;
    }
    for (int i = 0; i < file_names_.size(); ++i) {
        file_names_[i]  = directory + "/" + std::to_string(i) +".pcd";
    }
    std::cerr << "路径: " << directory << " 有" << file_names_.size() << "个pcd文件" << std::endl;
    return true;
}



void *mapping_thread(void *ptr ) {
    MappingParams *MappingParamPtr = (MappingParams*)ptr;
    mapping M;
    std::cout<<" 2 "<<MappingParamPtr->tid<<std::endl;
    M.init_mapping(MappingParamPtr->file_names_all,MappingParamPtr->tid);
    M.start_mapping();
    pthread_exit(NULL);
}

int main() {
    std::vector<std::string> file_names;
    std::string PcdPath = "/home/echo/pcd/pcd";
    GetFileNames(file_names,PcdPath);
    MappingParams *MappingParamPtr;

    float divided_num = file_names.size()/NUM_THREADS;

    for(int i=0; i < NUM_THREADS; i++ ) {
        int floor_num = floor(i * divided_num);
        int top_num = floor((i + 1) * divided_num);
        if (i == NUM_THREADS - 1) {
            top_num = file_names.size() -1;
        }
        tmp.file_names_all.push_back(std::vector<std::string>(file_names.begin() + floor_num, file_names.begin() + top_num));
        MappingParams temp;
        temp.tid = i;
        temp.file_names_all =tmp.file_names_all.back();
        ParamVector.push_back(temp);
    }

    pthread_t threads[NUM_THREADS];

    for(int i=0; i < NUM_THREADS; i++ ){
        MappingParamPtr = (MappingParams *) malloc(sizeof(ParamVector[i]));
        MappingParamPtr = &ParamVector[i];
        pthread_create(&threads[i], NULL, mapping_thread, (void *)MappingParamPtr);
    }
    pthread_exit(NULL);


    return 0;
}
