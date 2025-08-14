// util/ThreadHelpers.h
#pragma once
#include <atomic>
#include <afxwin.h>

// ������ ���Ḧ �����ϰ� ó���ϴ� �Լ�
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
