#include <CoreLayer/Math/Math.h>
#include <FunctionLayer/Camera/Pinhole.h>
#include <FunctionLayer/Integrator/Integrator.h>
#include <FunctionLayer/Sampler/Sampler.h>
#include <FunctionLayer/Scene/Scene.h>
#include <FunctionLayer/Texture/Mipmap.h>
#include <ResourceLayer/Factory.h>
#include <ResourceLayer/FileUtil.h>
#include <ResourceLayer/Image.h>
#include <ResourceLayer/JsonUtil.h>
#include <chrono>
#include <fstream>
#include <regex>
#include <stdio.h>
#include <CoreLayer/Debug/Debug.h>
#include <iomanip>
#include <sstream>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

std::chrono::time_point<std::chrono::system_clock> renderStartTime;
int totalPixels = 0;
std::atomic<int> processedPixels(0);
std::mutex progressMutex;

inline void printProgress(float percentage) {
  int val = (int)(percentage * 100);
  int lpad = (int)(percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  
  auto currentTime = std::chrono::system_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - renderStartTime).count();
  
  // 计算剩余时间
  float remainingTime = 0;
  if (percentage > 0) {
    remainingTime = (elapsed / percentage) * (1 - percentage);
  }
  
  // 格式化时间显示
  std::stringstream timeInfo;
  timeInfo << elapsed << "s";
  if (remainingTime > 0) {
    timeInfo << " | " << std::fixed << std::setprecision(1) << remainingTime << "s";
  }
  
  printf("\r%3d%% [%.*s%*s] %s", val, lpad, PBSTR, rpad, "", timeInfo.str().c_str());
  fflush(stdout);
}

void renderTile(int threadId, int numThreads, int width, int height, int spp, 
                std::shared_ptr<Camera> camera, std::shared_ptr<Scene> scene,
                std::shared_ptr<Integrator> integrator, std::shared_ptr<Sampler> sampler) {
  // 使用交错分配策略，每个线程处理间隔的行
  for (int y = threadId; y < height; y += numThreads) {
    for (int x = 0; x < width; ++x) {
      Vector2f NDC{(float)x / width, (float)y / height};
      Spectrum li(.0f);
      for (int i = 0; i < spp; ++i) {
        Ray ray = camera->sampleRayDifferentials(
            CameraSample{sampler->next2D()}, NDC);
        li += integrator->li(ray, *scene, sampler);
      }
      camera->film->deposit({x, y}, li / spp);

      processedPixels++;
      if (processedPixels % 5 == 0) {
        std::lock_guard<std::mutex> lock(progressMutex);
        printProgress((float)processedPixels / totalPixels);
      }
    }
  }
}

int main(int argc, char **argv) {
  // 初始化调试系统
  Debug::DebugManager::getInstance()->enableFileLogging("debug.log");

  // 解析命令行参数
  bool useMultiThread = true;
  if (argc > 2) {
    std::string arg = argv[2];
    if (arg == "--no-thread" || arg == "-n") {
      useMultiThread = false;
    }
  }

  const std::string sceneDir = std::string(argv[1]);
  FileUtil::setWorkingDirectory(sceneDir);
  std::string sceneJsonPath = FileUtil::getFullPath("scene.json");
  std::ifstream fstm(sceneJsonPath);
  Json json = Json::parse(fstm);
  auto camera = Factory::construct_class<Camera>(json["camera"]);
  auto scene = std::make_shared<Scene>(json["scene"]);
  auto integrator = Factory::construct_class<Integrator>(json["integrator"]);
  auto sampler = Factory::construct_class<Sampler>(json["sampler"]);
  int spp = sampler->xSamples * sampler->ySamples;
  int width = camera->film->size[0], height = camera->film->size[1];
  totalPixels = width * height;
  processedPixels = 0;
  renderStartTime = std::chrono::system_clock::now();

  if (useMultiThread) {
    unsigned int numThreads = std::thread::hardware_concurrency();
    std::cout << "Using " << numThreads << " threads" << std::endl;
    if (numThreads == 0) numThreads = 4;
    
    std::vector<std::thread> threads;
    
    // 创建并启动线程，使用交错分配策略
    for (unsigned int i = 0; i < numThreads; ++i) {
      threads.emplace_back(renderTile, i, numThreads, width, height, spp,
                          camera, scene, integrator, sampler);
    }
    
    // 等待所有线程完成
    for (auto& thread : threads) {
      thread.join();
    }
  } else {
    std::cout << "Using single thread" << std::endl;
    renderTile(0, 1, width, height, spp, camera, scene, integrator, sampler);
  }
  
  printProgress(1.f);

  auto end = std::chrono::system_clock::now();

  printf("\nRendering costs %.2fs\n",
         (std::chrono::duration_cast<std::chrono::milliseconds>(end - renderStartTime))
                 .count() /
             1000.f);

  //* 目前支持输出为png/hdr两种格式
  std::string outputName = fetchRequired<std::string>(json["output"], "filename");
  std::string outputDir = "output";
  // 确保输出目录存在
  FileUtil::setWorkingDirectory(outputDir);
  
  // 构建完整的输出路径
  std::string outputPath = FileUtil::getFullPath(outputName);
  
  if (std::regex_match(outputName, std::regex("(.*)(\\.png)"))) {
    camera->film->savePNG(outputPath.c_str());
  } else if (std::regex_match(outputName, std::regex("(.*)(\\.hdr)"))) {
    camera->film->saveHDR(outputPath.c_str());
  } else {
    std::cout << "Only support output as PNG/HDR\n";
  }
}
