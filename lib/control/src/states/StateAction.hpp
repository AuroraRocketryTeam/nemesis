#pragma once

#include "IStateAction.hpp"
#include <memory>
#include <vector>

/**
 * @brief Concrete implementation of IStateAction that manages rocket state actions and tasks.
 * 
 * The StateAction class provides a fluent interface for configuring state-specific behaviors
 * including entry/exit actions and associated tasks. It supports method chaining for
 * convenient configuration.
 * 
 * This class encapsulates:
 * - A specific rocket state
 * - Optional entry and exit actions (callbacks)
 * - A collection of task configurations associated with the state
 * 
 * The class uses std::function for flexible callback assignment and provides
 * const-correct access to internal data.
 */
class StateAction : public IStateAction
{
private:
    RocketState state;
    std::function<void()> entryAction;
    std::function<void()> exitAction;
    std::vector<TaskConfig> taskConfigs;

public:
    StateAction(RocketState s) : state(s) {}

    /**
     * @brief Set the entry action callback
     * @param action The function to call on state entry
     * @return Reference to this StateAction for method chaining
     */
    StateAction &setEntryAction(const std::function<void()> &action)
    {
        entryAction = action;
        return *this;
    }

    /**
     * @brief Set the exit action callback
     * @param action The function to call on state exit
     * @return Reference to this StateAction for method chaining
     */
    StateAction &setExitAction(const std::function<void()> &action)
    {
        exitAction = action;
        return *this;
    }

    /**
     * @brief Add a task configuration associated with this state
     * @param config The TaskConfig to add
     * @return Reference to this StateAction for method chaining
     */
    StateAction &addTask(const TaskConfig &config)
    {
        taskConfigs.push_back(config);
        return *this;
    }

    /**
     * @brief Called when entering this state
     */
    void onEntry() override
    {
        if (entryAction)
            entryAction();
    }

    /**
     * @brief Called when exiting this state
     */
    void onExit() override
    {
        if (exitAction)
            exitAction();
    }

    /**
     * @brief Get the state this action handles
     * @return The RocketState this action is associated with
     */
    RocketState getState() const override { return state; }

    /**
     * @brief Get the configured task configurations
     * @return A const reference to the vector of TaskConfig
     */
    const std::vector<TaskConfig> &getTaskConfigs() const { return taskConfigs; }
};