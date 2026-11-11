/*
 * A library with no work except to say, the moment a process starts, that it is there: the kernel
 * starts the programs /etc/inject.conf names with DYLD_INSERT_LIBRARIES in their environment
 * (kern.exec_inject, handed to the kernel by init.c's configure_inject), and bt2's run left a
 * question that wants the answer said from inside the process -- backboardd (RunAtLoad) had the
 * library of ours and said its lines, while bluetoothd and BlueTool (on-demand, user/501) said
 * neither, although the kernel had the whole list.  This one says the program's name, its pid and
 * the variable as the process itself sees it, before anything inside the program can fail quietly
 * (a library of ours returns early when it does not know the program, which its own lines cannot
 * tell apart from never having been inserted).  It goes into a program as a second library next to
 * libredwood or as the only one; whichever it is, a process it reached says so.  Said on stderr:
 * every job has the console there (mkjobs' StandardErrorPath).
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

__attribute__((constructor))
static void
probe(void)
{
	char line[768];
	const char *name = getprogname();
	const char *insert = getenv("DYLD_INSERT_LIBRARIES");
	int length = snprintf(line, sizeof(line), "probe: %s (pid %d): %s\n",
	    name != NULL ? name : "(no name)", getpid(),
	    insert != NULL ? insert : "DYLD_INSERT_LIBRARIES is not in the environment");

	if (length > 0) {
		write(2, line, (size_t)length < sizeof(line) ? (size_t)length : sizeof(line) - 1);
	}
}
