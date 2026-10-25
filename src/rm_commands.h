// rm_commands.h -- the ONE list of remote-management (RM1) commands, and the two text limits of the RM
// free-text commands. Arduino-free, C++11 (the firmware toolchains are not all C++14), header-only.
//
// Consumers derive their subset from RM_COMMAND_LIST:
//   RM_ALLOWLIST         src/remote_cmd.cpp   every row, with its argument shape
//   kToggles             src/rm_runtime.cpp   the TOGGLE rows (flag pointers stay there; checked at compile time)
//   read / write tables  src/rm_exec_read.cpp, src/rm_exec_write.cpp   READ + RW / RW rows (checked at compile time)
//   the Remote web page  a host harness parses this file and pins the page's command lists to it
//
// FORMAT CONTRACT (the jsdom harness reads this file with a regex, keep it parseable):
//   one entry per line, exactly  X(<name>, <KIND>, <SHAPE>)  with optional trailing backslash;
//   no preprocessor conditionals and no comments inside the list; the list ends at the first line
//   without a trailing backslash. Regex: /^\s*X\((\w+),\s*(\w+),\s*(\w+)\)/
//
// KIND   what the runtime does with the command
//   ACTION  no state flag: reboot status sendpos sendtrack sync
//   TOGGLE  "<name> on|off" (gps track display led gateway mesh); the console-flag ones are kToggles, led is a pin
//           (led takes on|off|flash)
//   PARAM   its own argument, handled in execute() (txpower setout chgled)
//   READ    answered by the read executor (radio sens txq mbox maxhop net, mh with an optional row/call)
//   RW      read without args, write with args: name atxt pos; upd ("" read, "check" / "go <tag>" act)
//           blockcall: BLK-01 call blocklist, "" reads, "none" or a comma list writes the WHOLE list. Over RM the
//           args are capped at RM_MAX_ARGS (39) chars, so a list set remotely holds about 3-4 calls
// SHAPE  suffix of RM_ARGS_<SHAPE> in remote_cmd.cpp (the argument check of the allowlist)
//
// Adding a command: one line here. The build then fails until the allowlist shape exists and every
// READ/RW row has a handler row (static_assert in rm_exec_read.cpp / rm_exec_write.cpp).
#ifndef RM_COMMANDS_H
#define RM_COMMANDS_H

#include <stddef.h>

#include "meshcom_settings.h"

#define RM_COMMAND_LIST(X)                                                                                      \
    X(reboot, ACTION, NONE)                                                                                      \
    X(status, ACTION, NONE)                                                                                      \
    X(sendpos, ACTION, NONE)                                                                                     \
    X(sendtrack, ACTION, NONE)                                                                                   \
    X(sync, ACTION, NONE)                                                                                        \
    X(gps, TOGGLE, ONOFF)                                                                                        \
    X(track, TOGGLE, ONOFF)                                                                                      \
    X(display, TOGGLE, ONOFF)                                                                                    \
    X(led, TOGGLE, LED)                                                                                          \
    X(gateway, TOGGLE, ONOFF)                                                                                    \
    X(mesh, TOGGLE, ONOFF)                                                                                       \
    X(txpower, PARAM, TXPOWER)                                                                                   \
    X(setout, PARAM, SETOUT)                                                                                     \
    X(chgled, PARAM, CHGLED)                                                                                     \
    X(radio, READ, NONE)                                                                                         \
    X(sens, READ, NONE)                                                                                          \
    X(txq, READ, NONE)                                                                                           \
    X(mbox, READ, NONE)                                                                                          \
    X(maxhop, READ, NONE)                                                                                        \
    X(name, RW, NAME)                                                                                            \
    X(atxt, RW, ATXT)                                                                                            \
    X(pos, RW, POS)                                                                                              \
    X(mh, READ, MH)                                                                                              \
    X(net, READ, NONE)                                                                                           \
    X(upd, RW, UPD)                                                                                              \
    X(blockcall, RW, CALLS)

enum RmCommandKind
{
    RM_CK_ACTION,
    RM_CK_TOGGLE,
    RM_CK_PARAM,
    RM_CK_READ,
    RM_CK_RW
};

// ---- text limits ----------------------------------------------------------------------------------------
// Derived from the settings struct, so a resized field moves every RM check with it (the page generator
// hands the same values to the page JS). test_rm_text pins today's values 19 / 39.
constexpr size_t RM_NAME_MAX = sizeof(s_meshcom_settings::node_name) - 1;
constexpr size_t RM_ATXT_MAX = sizeof(s_meshcom_settings::node_atxt) - 1;

// ---- compile-time queries over the list (C++11: single-expression constexpr, recursion) ---------------------
constexpr bool rmCmdNameEq(const char *a, const char *b)
{
    return *a == *b && (*a == '\0' || rmCmdNameEq(a + 1, b + 1));
}

// kind of the command `name` as an int, -1 when the list has no such command
#define RM_COMMAND_KIND_ROW(n, k, s) rmCmdNameEq(#n, name) ? (int)RM_CK_##k :
constexpr int rmCommandKind(const char *name)
{
    return RM_COMMAND_LIST(RM_COMMAND_KIND_ROW) - 1;
}
#undef RM_COMMAND_KIND_ROW

constexpr bool rmKindIsToggle(RmCommandKind k) { return k == RM_CK_TOGGLE; }
constexpr bool rmKindIsRead(RmCommandKind k) { return k == RM_CK_READ || k == RM_CK_RW; }
constexpr bool rmKindIsWrite(RmCommandKind k) { return k == RM_CK_RW; }

constexpr bool rmKindPasses(bool (*pred)(RmCommandKind), int kind)
{
    return kind >= 0 && pred((RmCommandKind)kind);
}

// number of commands in the list, and of those whose kind passes pred
#define RM_COMMAND_COUNT_ROW(n, k, s) +1
constexpr size_t RM_COMMAND_COUNT = 0 RM_COMMAND_LIST(RM_COMMAND_COUNT_ROW);
#undef RM_COMMAND_COUNT_ROW

#define RM_COMMAND_WHERE_ROW(n, k, s) +(pred(RM_CK_##k) ? 1 : 0)
constexpr size_t rmCountWhere(bool (*pred)(RmCommandKind))
{
    return 0 RM_COMMAND_LIST(RM_COMMAND_WHERE_ROW);
}
#undef RM_COMMAND_WHERE_ROW

// rows[i].name == name for some i >= from
template <typename Row, size_t N>
constexpr bool rmRowsHaveName(const Row (&rows)[N], const char *name, size_t from)
{
    return from < N && (rmCmdNameEq(rows[from].name, name) || rmRowsHaveName(rows, name, from + 1));
}

// every row from `from` on is a list command whose kind passes pred (and is not `except`)
template <typename Row, size_t N>
constexpr bool rmRowsInList(const Row (&rows)[N], bool (*pred)(RmCommandKind), const char *except, size_t from)
{
    return from >= N || ((except == nullptr || !rmCmdNameEq(rows[from].name, except)) &&
                         rmKindPasses(pred, rmCommandKind(rows[from].name)) &&
                         rmRowsInList(rows, pred, except, from + 1));
}

// every list command whose kind passes pred (except `except`) has a row
#define RM_COMMAND_COVER_ROW(n, k, s) ((!pred(RM_CK_##k) || (except != nullptr && rmCmdNameEq(#n, except))) || rmRowsHaveName(rows, #n, 0)) &&
template <typename Row, size_t N>
constexpr bool rmRowsCoverList(const Row (&rows)[N], bool (*pred)(RmCommandKind), const char *except)
{
    return RM_COMMAND_LIST(RM_COMMAND_COVER_ROW) true;
}
#undef RM_COMMAND_COVER_ROW

// The rows are EXACTLY the list commands whose kind passes pred, minus the optional `except`:
// nothing foreign, nothing missing, no duplicate (the size check closes the last gap).
template <typename Row, size_t N>
constexpr bool rmRowsMatchList(const Row (&rows)[N], bool (*pred)(RmCommandKind), const char *except = nullptr)
{
    return rmRowsInList(rows, pred, except, 0) && rmRowsCoverList(rows, pred, except) &&
           N + (except != nullptr && rmKindPasses(pred, rmCommandKind(except)) ? 1 : 0) == rmCountWhere(pred);
}

#endif // RM_COMMANDS_H
