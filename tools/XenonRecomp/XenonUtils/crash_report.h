#pragma once

// Installs a crash reporter for the host code-generation tools. On a fatal
// signal (SIGSEGV/SIGBUS/SIGFPE/SIGABRT) it prints the caught signal and a
// native backtrace to stderr, then exits with a non-zero code. These tools
// run unattended in CI, so a backtrace in the build log is the fastest way
// to locate a crash. No-op on Windows (where the crash dialog serves the
// same purpose).
void InstallCrashReporter();
