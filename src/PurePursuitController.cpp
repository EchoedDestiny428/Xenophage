// move this into main when you are done testing.
// dont expect this to be finished
// Osi don't stres over this.

#include <vector>
#include <iostream>
#include <cmath>

class PurePursuitController {
    public:
        struct Pose {
            float x;
            float y;
        } currentPose;


        void setPose(float x, float y, float heading) {
            currentPose.x = x;
            currentPose.y = y;
        }

        void addPathPoint(float x, float y) {
            path.push_back({x, y});
        }

        void setLookAheadDistance(float distance) {
            lookAheadDistance = distance;
        }

        int sign(float number) {
            if (number >= 0) {
                return 1;
            } else {
                return -1;
            }
        }


        std::vector<Pose> line_circle_intersection(std::vector<Pose> currentPos, std::vector<Pose> pt1, std::vector<Pose> pt2, float lookAheadDistance) {
            bool intersectFound = false;

            float currentX = currentPos[0].x;
            float currentY = currentPos[0].y;
            float x1 = pt1[0].x;
            float y1 = pt1[0].y;
            float x2 = pt2[0].x;
            float y2 = pt2[0].y; 

            float x1_offset = x1 - currentX;
            float y1_offset = y1 - currentY;
            float x2_offset = x2 - currentX;
            float y2_offset = y2 - currentY;

            float dx = x2_offset - x1_offset;
            float dy = y2_offset - y1_offset;
            float dr = std::sqrt(pow(dx, 2) + pow(dy, 2));
            float D = x1_offset*y2_offset - x2_offset*y1_offset;
            float discriminant = (pow(lookAheadDistance, 2)) * (pow(dr, 2)) - pow(D, 2);

            // Check if the line intersects with the circle
            float discriminant = b * b - 4 * a * c;
            
            if (line_circle_intersection(currentPose.x, currentPose.y, path[currentPathIndex].x, path[currentPathIndex].y, currentPose.x, currentPose.y) >= 0) {
                sol_x1 = (D * dy + sign(dy) * dx * np.sqrt(discriminant)) / dr**2;
                sol_x2 = (D * dy - sign(dy) * dx * np.sqrt(discriminant)) / dr**2;
                sol_y1 = (- D * dx + abs(dy) * np.sqrt(discriminant)) / dr**2;
                sol_y2 = (- D * dx - abs(dy) * np.sqrt(discriminant)) / dr**2 ; 
            } else {
                return;
            }

        
            return;
        }

    private:
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