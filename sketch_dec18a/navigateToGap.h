#ifndef NAVIGATETOGAP_H
#define NAVIGATETOGAP_H

// 导航状态枚举（需要在主文件中使用）
enum NavigationState {
  NAV_IDLE,           // 空闲，等待新的路径规划
  NAV_TURN_TO_GAP,   // 转向空洞方向
  NAV_MOVE_TO_ENTRANCE, // 移动到洞口入口
  NAV_TURN_BACK,     // 转回正前方
  NAV_MOVE_THROUGH,  // 通过空洞
  NAV_SEARCHING      // 搜索模式：未找到空洞或检测到碰撞时，旋转搜索
};

// 搜索原因枚举
enum SearchReason {
  SEARCH_NO_REASON = 0,    // 无原因
  SEARCH_NO_GAP = 1,       // S:1 - 未找到空洞
  SEARCH_COLLISION = 2     // S:2 - 检测到近距离障碍物
};

void runGapTest();
void updateNavigation();  // 更新导航状态（需要在主循环中持续调用）
NavigationState getNavigationState();  // 获取当前导航状态
SearchReason getSearchReason();  // 获取当前搜索原因

// 获取下一步执行步骤信息（用于屏幕显示）
void getNextStepInfo(char* step1, float* value1, char* step2, float* value2, char* step3, float* value3);
// 获取完整计划步骤（用于测试模式显示）
int getPlannedStepsForDisplay(char steps[][10], float values[], int maxSteps);
// 步进暂停控制（电机启用时每步执行完暂停，按键继续）
bool isStepPauseActive();
void resumeStepPause();
bool isStepPauseModeEnabled();
void toggleStepPauseMode();

#endif
