#include "game_theory_sim/game_theory_planner.h"
#include <algorithm>

const double INF_COST = 1e9;

double pathLength(const Path& p) {
    if (p.size() < 2) return 0.0;
    double len = 0.0;
    for (size_t i = 0; i < p.size() - 1; ++i) {
        double dx = p[i+1].x - p[i].x;
        double dy = p[i+1].y - p[i].y;
        len += std::sqrt(dx*dx + dy*dy);
    }
    return len;
}

void computePayoffMatrix(Agent& a1, Agent& a2, std::vector<std::vector<double>>& M1, std::vector<std::vector<double>>& M2) {
    double r_sum = a1.Radius + a2.Radius;
    int n1 = a1.StrategySet.size();
    int n2 = a2.StrategySet.size();

    M1.assign(n1, std::vector<double>(n2, 0.0));
    M2.assign(n1, std::vector<double>(n2, 0.0));

    std::vector<double> len1(n1), len2(n2);
    for (int i = 0; i < n1; ++i) {
        double l = pathLength(a1.StrategySet[i]);
        len1[i] = (l < 1e-3) ? a1.StandStillCost : l;
    }
    for (int j = 0; j < n2; ++j) {
        double l = pathLength(a2.StrategySet[j]);
        len2[j] = (l < 1e-3) ? a2.StandStillCost : l;
    }

    for (int i = 0; i < n1; ++i) {
        for (int j = 0; j < n2; ++j) {
            const Path& p1 = a1.StrategySet[i];
            const Path& p2 = a2.StrategySet[j];
            int T = std::max(p1.size(), p2.size());
            
            bool collision = false;
            for (int t = 0; t < T; ++t) {
                Point2D pt1 = (t < p1.size()) ? p1[t] : p1.back();
                Point2D pt2 = (t < p2.size()) ? p2[t] : p2.back();
                double dx = pt1.x - pt2.x;
                double dy = pt1.y - pt2.y;
                if (std::sqrt(dx*dx + dy*dy) < r_sum) {
                    collision = true;
                    break;
                }
            }

            if (collision) {
                M1[i][j] = INF_COST;
                M2[i][j] = INF_COST;
            } else {
                M1[i][j] = len1[i];
                M2[i][j] = len2[j];
            }
        }
    }
}

std::pair<int, int> findNashEquilibrium(const std::vector<std::vector<double>>& M1, const std::vector<std::vector<double>>& M2) {
    int rows = M1.size();
    int cols = M1[0].size();
    std::vector<std::pair<int, int>> candidates;

    // Step 1: Nash candidates (Def 2)
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // Find min in column j for A1
            double minA1 = INF_COST;
            for(int r = 0; r < rows; r++) minA1 = std::min(minA1, M1[r][j]);
            
            // Find min in row i for A2
            double minA2 = INF_COST;
            for(int c = 0; c < cols; c++) minA2 = std::min(minA2, M2[i][c]);

            if (M1[i][j] == minA1 && M2[i][j] == minA2) {
                candidates.push_back({i, j});
            }
        }
    }

    if (candidates.empty()) {
        // Fallback: Min-Sum
        double minSum = INF_COST * 2;
        std::pair<int, int> best = {0, 0};
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                if (M1[i][j] + M2[i][j] < minSum) {
                    minSum = M1[i][j] + M2[i][j];
                    best = {i, j};
                }
            }
        }
        return best;
    }

    // Step 2: Pareto Filtering
    std::vector<bool> dominated(candidates.size(), false);
    for (size_t k = 0; k < candidates.size(); ++k) {
        if (dominated[k]) continue;
        double c1k = M1[candidates[k].first][candidates[k].second];
        double c2k = M2[candidates[k].first][candidates[k].second];
        
        for (size_t l = 0; l < candidates.size(); ++l) {
            if (k == l || dominated[k]) continue;
            double c1l = M1[candidates[l].first][candidates[l].second];
            double c2l = M2[candidates[l].first][candidates[l].second];
            
            if (c1l <= c1k && c2l <= c2k && (c1l < c1k || c2l < c2k)) {
                dominated[k] = true;
            }
        }
    }

    std::vector<std::pair<int, int>> pareto_set;
    for (size_t k = 0; k < candidates.size(); ++k) {
        if (!dominated[k]) pareto_set.push_back(candidates[k]);
    }

    // Step 3: Random selection among Pareto-optimal
    srand(time(NULL));
    int rand_idx = rand() % pareto_set.size();
    return pareto_set[rand_idx];
}

// Generazione mock delle traiettorie (Da espandere con la tua logica Unicycle RRT)
void Agent::plan(Environment& env, int numPaths) {
    StrategySet.clear();
    
    // In C++ puro, qui chiamerai multiTrajectoryRRT(). 
    // Per brevità in questo esempio, generiamo una traiettoria dritta verso il goal
    Path direct_path;
    direct_path.push_back(Position);
    direct_path.push_back(Goal);
    StrategySet.push_back(direct_path);
    
    // Aggiungi la PrevNashPath
    if (!PrevNashPath.empty()) {
        StrategySet.push_back(PrevNashPath);
    }
    
    // Azione di default: Stand Still
    Path standStill = {Position, Position};
    StrategySet.push_back(standStill);
    StandStillCost = pathLength(direct_path) * 1.5; 
}
