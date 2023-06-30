#ifndef STRTOK_H
#define STRTOK_H

char *
strtok_r (register char *__restrict s,
	register const char *__restrict delim,
	char **__restrict lasts);

#endif
