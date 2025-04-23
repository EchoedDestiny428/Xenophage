// move this into main when you are done testing.
// dont screw up, Osiris.

#include <vector>
#include <iostream>
#include <cmath>

class PurePursuitController {
    public:
        void setPose(float x, float y, float heading) {
            currentPose.x = x;
            currentPose.y = y;
        }

        void addPathPoint(float x, float y) {
            path.push_back({x, y});
        }

    private:
        struct Pose {
            float x;
            float y;
        } currentPose;

        int currentPathIndex = 0;

        std::vector<Pose> path;

        float lookAheadDistance = 3; // inches
};

int main() {
    PurePursuitController controller;
    controller.setPose(0, 0, 0);

    std::cout << "test" << std::endl;

    return 1;
}