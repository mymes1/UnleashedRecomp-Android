#pragma once

// Installs a crash reporter for the host code-generation tools. On a fatal
// signal (SIGSEGV/SIGBUS/SIGFPE/SIGABRT) it prints the caught signal and a
// native backtrace to stderr, then exits with a non-zero code. These tools
// run unattended in CI, so a backtrace in the build log is the fastest way
// to locate a crash. Enabled on glibc/BSD only: it is a no-op on Windows
// (the crash dialog serves the same purpose) and on Android (bionic has no
// <execinfo.h>, and this file is cross-compiled into the app build where
// the reporter is never needed).
void InstallCrashReporter();
