#pragma once


class GameTimer
{
public:
	GameTimer();

	void Tick();

	void Reset();
	void Start();
	void Stop();

	float DeltaTime() const;
	float TotalTime() const;

private:
	INT64 mBaseTime, mPausedTime, mStopTime, mCurrTime, mPrevTime;

	float mDeltaTime, mSecondsPerTick;

	bool mIsStopped;

};