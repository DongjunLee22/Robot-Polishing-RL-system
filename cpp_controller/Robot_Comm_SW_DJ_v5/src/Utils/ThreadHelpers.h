// util/ThreadHelpers.h
#pragma once
#include <atomic>
#include <afxwin.h>

// 쓰레드 종료를 안전하게 처리하는 함수
inline void SafeThreadQuit(CWinThread*& pTh,
    HANDLE& hTh,
    std::atomic<bool>& runFlag,
    DWORD waitMs = 2000)
{
    if (pTh) {
        runFlag.store(false);
        if (hTh) ::WaitForSingleObject(hTh, waitMs);
        pTh = nullptr; hTh = nullptr;
    }
}
