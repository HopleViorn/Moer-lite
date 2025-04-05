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

void renderTile(int startY, int endY, int width, int height, int spp, 
                std::shared_ptr<Camera> camera, std::shared_ptr<Scene> scene,
                std::shared_ptr<Integrator> integrator, std::shared_ptr<Sampler> sampler) {
  for (int y = startY; y < endY; ++y) {
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

  unsigned int numThreads = std::thread::hardware_concurrency();
  std::cout << "numThreads: " << numThreads << std::endl;
  if (numThreads == 0) numThreads = 4;
  
  int rowsPerThread = height / numThreads;
  std::vector<std::thread> threads;
  
  for (unsigned int i = 0; i < numThreads; ++i) {
    int startY = i * rowsPerThread;
    int endY = (i == numThreads - 1) ? height : (i + 1) * rowsPerThread;
    threads.emplace_back(renderTile, startY, endY, width, height, spp,
                        camera, scene, integrator, sampler);
  }
  
  for (auto& thread : threads) {
    thread.join();
  }
  
  printProgress(1.f);

  auto end = std::chrono::system_clock::now();

  printf("\nRendering costs %.2fs\n",
         (std::chrono::duration_cast<std::chrono::milliseconds>(end - renderStartTime))
                 .count() /
             1000.f);

  //* 目前支持输出为png/hdr两种格式
  std::string outputName =
      fetchRequired<std::string>(json["output"], "filename");
  if (std::regex_match(outputName, std::regex("(.*)(\\.png)"))) {
    camera->film->savePNG(outputName.c_str());
  } else if (std::regex_match(outputName, std::regex("(.*)(\\.hdr)"))) {
    camera->film->saveHDR(outputName.c_str());
  } else {
    std::cout << "Only support output as PNG/HDR\n";
  }
}
