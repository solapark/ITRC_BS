#pragma once
#include "stdafx.h"

#ifndef _timeStamp_H
#define _timeStamp_H

#include <Windows.h>
#include <cstdint>

class timeStamp {
public:
	void updateT(const SYSTEMTIME &now);
	void getCurT(uint64_t &t) const;
	void getTDiff(uint64_t & TDiff) const;

private:
	SYSTEMTIME curT, pastT;
};


#endif // !_timeStamp_H