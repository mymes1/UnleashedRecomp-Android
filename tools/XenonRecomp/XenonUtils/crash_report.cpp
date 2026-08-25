#include "crash_report.h"

// The backtrace implementation relies on <execinfo.h> (backtrace /
// backtrace_symbols_fd), which exists on glibc/BSD but NOT on Android
// (bionic). This file is also compiled into the Android build (UnleashedRecomp
// links XenonUtils at runtime), so the reporter is only enabled on the
// native build host, where the code-generation tools actually run.
#if defined(__linux__) && !defined(__ANDROID__)

#include <csignal>
#include <setjmp.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <execinfo.h>

namespace
{
    volatile sig_atomic_t g_CrashSignal = 0;
    sigjmp_buf g_CrashJmpBuffer;

    void CrashReporterHandler(int sig)
    {
        if (g_CrashSignal)
        {
            // Crashed while reporting a crash; restore the default handler
            // and re-raise so the core file is not lost.
            signal(sig, SIG_DFL);
            raise(sig);
        }

        g_CrashSignal = sig;
        siglongjmp(g_CrashJmpBuffer, 1);
    }

    void PrintBacktrace(int sig)
    {
        void* frames[64];
        int count = backtrace(frames, 64);

        fprintf(stderr, "\n=== Host tool caught fatal signal %d; backtrace:\n", sig);
        // backtrace_symbols_fd is write()-based, so it is safe to call here.
        backtrace_symbols_fd(frames, count, STDERR_FILENO);
    }
}

void InstallCrashReporter()
{
    // Returning here (nonzero) means a fatal signal was caught: print the
    // backtrace from a safe context, then exit non-zero.
    if (sigsetjmp(g_CrashJmpBuffer, 1) != 0)
    {
        PrintBacktrace(static_cast<int>(g_CrashSignal));
        _exit(128 + static_cast<int>(g_CrashSignal));
    }

    signal(SIGSEGV, CrashReporterHandler);
    signal(SIGBUS, CrashReporterHandler);
    signal(SIGFPE, CrashReporterHandler);
    signal(SIGABRT, CrashReporterHandler);
}

#else

void InstallCrashReporter()
{
}

#endif
