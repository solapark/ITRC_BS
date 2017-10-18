#include "stdafx.h"
#include "timeStamp.h"


void timeStamp::updateT(const SYSTEMTIME &now)
{
	pastT = curT;
	curT = now;
}

void timeStamp::getCurT(uint64_t &t) const
{
	t =
		(curT.wYear % 2000) * (uint64_t)10000000000000
		+ curT.wMonth * (uint64_t)100000000000
		+ curT.wDay * (uint64_t)1000000000
		+ curT.wHour * 10000000
		+ curT.wMinute * 100000
		+ curT.wSecond * 1000
		+ curT.wMilliseconds
		;

}
void timeStamp::getIntervalT(uint64_t & tDiff) const
{
	union timeunion {
		FILETIME fileTime;
		ULARGE_INTEGER ul;
	};

	timeunion ft1;
	timeunion ft2;

	SystemTimeToFileTime(&pastT, &ft1.fileTime);
	SystemTimeToFileTime(&curT, &ft2.fileTime);

	tDiff =( ft2.ul.QuadPart - ft1.ul.QuadPart) / 10000;
}
