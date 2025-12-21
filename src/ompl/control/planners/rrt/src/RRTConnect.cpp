/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include "ompl/control/planners/rrt/RRTConnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::control::RRTConnect::RRTConnect(const SpaceInformationPtr &si) : base::Planner(si, "RRTConnect")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &RRTConnect::setGoalBias, &RRTConnect::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RRTConnect::setIntermediateStates, &RRTConnect::getIntermediateStates,
                                "0,1");
    
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

ompl::control::RRTConnect::~RRTConnect()
{
    freeMemory();
}

void ompl::control::RRTConnect::setup()
{
    base::Planner::setup();
    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::control::RRTConnect::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    lastGoalMotion_ = nullptr;
}

void ompl::control::RRTConnect::freeMemory()
{
    std::vector<Motion *> motions;
    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

ompl::control::RRTConnect::GrowState ompl::control::RRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                         Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;
    Motion *xmotion = new Motion(siC_);
    siC_->copyState(xmotion->state, rmotion->state);

    unsigned int cd = controlSampler_->sampleTo(xmotion->control, nmotion->control, nmotion->state, xmotion->state);

    bool validMotion = cd >= siC_->getMinControlDuration() && distanceFunction(nmotion, xmotion) >= minDistance_;

    if (!validMotion)
        return TRAPPED;
    
    if (addIntermediateStates_)
    {
        std::vector<base::State *> pstates;
        siC_->propagateWhileValid(nmotion->state, xmotion->control, cd, pstates, true);

        Motion *lastmotion = nmotion;
        reach = false;
        size_t p = 0;
        for (; p < pstates.size(); ++p) {
            auto *motion = new Motion();
            motion->state = pstates[p];
            motion->control = siC_->allocControl();
            siC_->copyControl(motion->control, xmotion->control);
            motion->steps = 1;
            motion->parent = lastmotion;
            motion->root = lastmotion->root;
            lastmotion = motion;
            tree->add(motion);
            reach = distanceFunction(motion, rmotion) <= minDistance_;
            if (reach) {
                break;
            }
        }
        tgi.xmotion = lastmotion;

        while (++p < pstates.size())
            si_->freeState(pstates[p]);
        
        if (xmotion->state)
            si_->freeState(xmotion->state);
        if (xmotion->control)
            siC_->freeControl(xmotion->control);
        delete xmotion;
    } else
    {
        xmotion->steps = cd;
        xmotion->parent = nmotion->parent;
        xmotion->root = nmotion->root;
        tree->add(xmotion);

        tgi.xmotion = xmotion;
        reach = distanceFunction(xmotion, rmotion) <= minDistance_;
    }
    return reach ? REACHED : ADVANCED;
}

ompl::base::PlannerStatus ompl::control::RRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
    auto cspace = siC_->getControlSpace();

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        motion->root = motion->state;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    bool solved = false;
    base::PlannerStatus::StatusType status = base::PlannerStatus::TIMEOUT;

    while (ptc == false)
    {
        TreeData &tree = startTree_ ? tStart_ : tGoal_;
        tgi.start = startTree_;
        startTree_ = !startTree_;
        TreeData &otherTree = startTree_ ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, st);
                siC_->nullControl(motion->control);
                motion->root = motion->state;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                status = base::PlannerStatus::INVALID_GOAL;
                break;
            }
        }

        /* sample random state */
        sampler_->sampleUniform(rstate);

        GrowState gs = growTree(tree, tgi, rmotion);

        if (gs != TRAPPED)
        {
            Motion *addedMotion = tgi.xmotion;

            tgi.start = startTree_;
            GrowState gsc = growTree(otherTree, tgi, addedMotion);
            for (int i = 0; i < 5 && gsc == ADVANCED; ++i)
                gsc = growTree(otherTree, tgi, addedMotion);
            
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_) {
                distanceBetweenTrees_ = newDist;
            }

            Motion *startMotion = tgi.start ? tgi.xmotion : addedMotion;
            Motion *goalMotion = tgi.start ? addedMotion : tgi.xmotion;

            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                Motion *solution = startMotion;
                std::vector<Motion *> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion *> mpath2;
                while (solution != nullptr)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                auto path(std::make_shared<PathControl>(si_));
                path->getStates().reserve(mpath1.size() + mpath2.size());
                path->getControls().reserve(mpath1.size() + mpath2.size() - 1);
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    if (mpath1[i]->parent)
                        path->append(mpath1[i]->state, mpath1[i]->control, mpath1[i]->steps * siC_->getPropagationStepSize());
                    else
                        path->append(mpath1[i]->state);
                path->append(mpath2[0]->state, mpath2[0]->control, 0);
                for (int i = 1; i < mpath2.size(); ++i) {
                    siC_->copyControl(rctrl, mpath2[i-1]->control);
                    for (int j = 0; cspace->getValueAddressAtIndex(rctrl, j); ++j) {
                        auto elem = cspace->getValueAddressAtIndex(rctrl, j);
                        *elem = -(*elem);
                    }
                    path->append(mpath2[i]->state, rctrl, mpath2[i-1]->steps * siC_->getPropagationStepSize());
                }
                pdef_->addSolutionPath(path, false, 0.0, getName());
                solved = true;
                break;
            }
            else
            {
                if (tgi.start) {
                    double dist = 0.0;
                    goal->isSatisfied(tgi.xmotion->state, &dist);
                    if (dist < approxdif) {
                        approxdif = dist;
                        approxsol = tgi.xmotion;
                    }
                }
            }
        }
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    if (approxsol && !solved)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (approxsol != nullptr)
        {
            mpath.push_back(approxsol);
            approxsol = approxsol->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, true, approxdif, getName());
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    return solved ? base::PlannerStatus::EXACT_SOLUTION : status;
}

void ompl::control::RRTConnect::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    double delta = siC_->getPropagationStepSize();

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state, 1), base::PlannerDataVertex(m->state, 1),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state, 1), base::PlannerDataVertex(m->state, 1));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state, 1));
    }

    motions.clear();

    if (tGoal_)
        tGoal_->list(motions);

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->state, 2), base::PlannerDataVertex(m->parent->state, 2),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->state, 2), base::PlannerDataVertex(m->parent->state, 2));
        }
        else
            data.addGoalVertex(base::PlannerDataVertex(m->state, 2));
    }

    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
    data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}
