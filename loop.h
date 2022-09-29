#ifndef LOOP_H
#define LOOP_H

class Loop
{
public:

    /**
     * @brief
     * Sets the interval (in seconds).
     * @param interval
     * The interval (in seconds).
     */
    void SetInterval(double interval) { this->interval = interval;}

    /**
     * @brief
     * Gets the interval (in seconds).
     * @return
     * The interval (in seconds).
     */
    double GetInterval() const { return this->interval;}

    /**
     * @brief
     * Indicates whether the main loop is running.
     * @retval true
     * The main loop is running.
     * @retval false
     * The main loop has ended.
     */
    bool IsRunning() const { return this->running; }

    /**
     * @brief
     * Tells the main loop to exit as soon as it can.
     */
    void NotifyExit() {this->exit = true;}



protected:

    bool exit;
    bool running;
    double interval;
};

#endif // LOOP_H
