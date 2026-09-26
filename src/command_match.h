/**
 * How a serial/BLE command line is matched against a command name (D2-10).
 *
 * Carved out of command_functions.cpp so it can be tested on the host: that
 * file pulls in Arduino, LVGL and the radio stack, so no `env:native*`
 * compiles it, and the matching rule -- the thing the whole command ladder
 * rests on -- had no executable test at all. Everything here is pure C++ with
 * no Arduino dependency, exactly so `native_command_match` can exercise it.
 *
 * THE RULE
 * --------
 * Matching used to be a plain PREFIX comparison: the input was truncated to
 * the candidate's length and compared. Every name that was a prefix of a
 * later name therefore shadowed it, and the ONLY thing keeping the two apart
 * was the order the rungs happened to appear in. That is not hypothetical:
 * `softser app` ran for every `--softser app0` (so `iNextTelemetry` was never
 * reset), and `--msgid` would have executed as `--msg on` if its rung had not
 * been placed above `msg` by hand. test/golden/command_ladder_lint.py exists
 * only to police that ordering.
 *
 * That order-dependence has to go before D2-06/D2-07 can turn the ladder into
 * a table, because a table has no order to rely on. The rule below is the one
 * the ladder already followed informally, now enforced:
 *
 *   `command` ENDS IN A SPACE -> argument form, prefix match.
 *       "setname " matches "setname Martin". Everything past the space is the
 *       argument, so the prefix IS the whole contract and there is nothing
 *       left to terminate.
 *
 *   otherwise                 -> exact token.
 *       The name must end at the end of the line, or at the separator before
 *       its argument. So "msg" no longer matches "msgid", "pos" no longer
 *       matches "posshot" and "instr" no longer matches "instreset", while
 *       "--setctry 1" still matches "setctry" because a space terminates it.
 *
 * A short input can never match a longer command. The old code got that by
 * writing a terminator past the input's end into uninitialised stack (see the
 * audit appendix D2: it is what kept `balledges` safe next to `balledge on`);
 * here it is an explicit length check.
 *
 * Bench-checked on DK5EN-93 2026-09-16 with the D2-10 image: `--posx`,
 * `--infox` and `--msgidx` are all rejected, `--pos` still answers. Under the
 * old rule `--posx` executed `--pos`.
 */
#ifndef COMMAND_MATCH_H
#define COMMAND_MATCH_H

#include <ctype.h>
#include <string.h>

// Case-insensitive compare, unchanged in behaviour from the copy that lived
// in command_functions.cpp. Returns 0 when equal, like strcmp.
inline int casecmp(const char *s1, const char *s2)
{
	while (*s1 != 0 && tolower((unsigned char)*s1) == tolower((unsigned char)*s2))
	{
		++s1;
		++s2;
	}

	return (*s2 == 0) ? (*s1 != 0) : -1;
}

// True when the command line `msg` invokes `command`, per THE RULE above.
// `msg` is the line with its leading "--" already stripped, which is how
// every call site passes it (`msg_text + 2`).
inline bool commandMatches(const char *msg, const char *command)
{
	const size_t clen = strlen(command);
	const size_t mlen = strlen(msg);

	// A short input can never match a longer command.
	if (mlen < clen)
		return false;

	// Compare only the first clen characters, without copying: casecmp stops
	// at the end of its second argument, so a temporary terminator is not
	// needed the way the old fixed 100-byte buffer required.
	for (size_t i = 0; i < clen; ++i)
	{
		if (tolower((unsigned char)msg[i]) != tolower((unsigned char)command[i]))
			return false;
	}

	// Argument form: the trailing space already separated name from value.
	if (clen > 0 && command[clen - 1] == ' ')
		return true;

	// Exact token: the name must end here.
	const char tail = msg[clen];
	return tail == '\0' || tail == ' ' || tail == '\r' || tail == '\n';
}

#endif // COMMAND_MATCH_H
