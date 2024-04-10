// 写一个算法，实现动态维护滑动窗口中元素的最大值 
// 例如，滑动窗口的大小为3，输入数组为[1,3,-1,-3,5,3,6,7]，那么输出数组为[3,3,5,5,6,7]

#include <iostream>
#include <vector>
#include <deque>
using namespace std;

class Solution {
public:
    vector<int> maxSlidingWindow(vector<int>& nums, int k) {
        vector<int> res;
        deque<int> q;
        for (int i = 0; i < nums.size(); i++) {
            if (!q.empty() && q.front() == i - k) {
                q.pop_front();
            }
            while (!q.empty() && nums[q.back()] < nums[i]) {
                q.pop_back();
            }
            q.push_back(i);
            if (i >= k - 1) {
                res.push_back(nums[q.front()]);
            }
        }
        return res;
    }
};