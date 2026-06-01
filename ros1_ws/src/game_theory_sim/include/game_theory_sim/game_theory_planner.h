#ifndef GAME_THEORY_PLANNER_H
#define GAME_THEORY_PLANNER_H

#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include <tuple>
#include <iostream>

struct Point2D {
    double x, y;
};

struct State3D {
    double x, y, theta;
};

typedef std::vector<Point2D> Path;

// Struttura semplificata dell'Ambiente (Occupancy Grid)
class Environment {
public:
    double xLim[2] = {0.0, 20.0};
    double yLim[2] = {0.0, 20.0};
    
    // Funzione mock per il controllo collisioni (da integrare con nav_msgs/OccupancyGrid)
    bool isSegmentFree(const Point2D& p1, const Point2D& p2) {
        // Implementa qui il controllo collisioni con gli ostacoli (es. i muri definiti nel tuo main)
        // Per ora ritorna sempre true (libero)
        return true; 
    }
};

class Agent {
public:
    int ID;
    Point2D Position;
    Point2D Goal;
    double Radius = 0.300;
    double Velocity = 0.69;
    double Orientation;
    
    std::vector<Path> StrategySet;
    Path CurrentPath;
    Path PrevNashPath;
    double StandStillCost;

    Agent(int id, Point2D start, Point2D goal) : ID(id), Position(start), Goal(goal) {
        Orientation = atan2(goal.y - start.y, goal.x - start.x);
    }

    bool hasReachedGoal() {
        double dx = Position.x - Goal.x;
        double dy = Position.y - Goal.y;
        return std::sqrt(dx*dx + dy*dy) < 0.5;
    }

    void plan(Environment& env, int numPaths);
};

// Funzioni core
double pathLength(const Path& p);
void computePayoffMatrix(Agent& a1, Agent& a2, std::vector<std::vector<double>>& M1, std::vector<std::vector<double>>& M2);
std::pair<int, int> findNashEquilibrium(const std::vector<std::vector<double>>& M1, const std::vector<std::vector<double>>& M2);
std::vector<Path> multiTrajectoryRRT(Environment& env, Point2D start, Point2D goal, int numPaths, double v, double theta0);

#endif
