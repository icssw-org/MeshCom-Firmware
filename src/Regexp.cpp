#include <setjmp.h>
#include <ctype.h>
#include <string.h>
#include "Regexp.h"

// for throwing errors
//static
jmp_buf regexp_error_return;
typedef unsigned char byte;

// error codes raised during regexp processing
//static
byte error (/*const*/ char err)
{
  // does not return
  longjmp (regexp_error_return, err);
  return 0;  // keep compiler happy
}  // end of error

//static
int check_capture (MatchState *ms, int l) {
  l -= '1';
  if (l < 0 || l >= ms->level || ms->capture[l].len == CAP_UNFINISHED)
    return error(ERR_INVALID_CAPTURE_INDEX);
  return l;
} // end of check_capture

//static
int capture_to_close (MatchState *ms) {
  int level = ms->level;
  for (level--; level>=0; level--)
    if (ms->capture[level].len == CAP_UNFINISHED) return level;
  return error(ERR_INVALID_PATTERN_CAPTURE);
} // end of capture_to_close

//static const
char *classend (MatchState *ms, /*const*/ char *p) {
  switch (*p++) {
    case REGEXP_ESC: {
      if (*p == '\0')
        error(ERR_MALFORMED_PATTERN_ENDS_WITH_ESCAPE);
      return p+1;
    }
    case '[': {
      if (*p == '^') p++;
      do {  /* look for a `]' */
        if (*p == '\0')
          error(ERR_MALFORMED_PATTERN_ENDS_WITH_RH_SQUARE_BRACKET);
        if (*(p++) == REGEXP_ESC && *p != '\0')
          p++;  /* skip escapes (e.g. `%]') */
      } while (*p != ']');
      return p+1;
    }
    default: {
      return p;
    }
  }
} // end of classend


//static
int match_class (int c, int cl) {
  int res;
  switch (tolower(cl)) {
    case 'a' : res = isalpha(c); break;
    case 'c' : res = iscntrl(c); break;
    case 'd' : res = isdigit(c); break;
    case 'l' : res = islower(c); break;
    case 'p' : res = ispunct(c); break;
    case 's' : res = isspace(c); break;
    case 'u' : res = isupper(c); break;
    case 'w' : res = isalnum(c); break;
    case 'x' : res = isxdigit(c); break;
    case 'z' : res = (c == 0); break;
    default: return (cl == c);
  }
  return (islower(cl) ? res : !res);
} // end of match_class


//static
int matchbracketclass (int c, /*const*/ char *p, /*const*/ char *ec) {
  int sig = 1;
  if (*(p+1) == '^') {
    sig = 0;
    p++;  /* skip the `^' */
  }
  while (++p < ec) {
    if (*p == REGEXP_ESC) {
      p++;
      if (match_class(c, uchar(*p)))
        return sig;
    }
    else if ((*(p+1) == '-') && (p+2 < ec)) {
      p+=2;
      if (uchar(*(p-2)) <= c && c <= uchar(*p))
        return sig;
    }
    else if (uchar(*p) == c) return sig;
  }
  return !sig;
} // end of matchbracketclass


//static
int singlematch (int c, /*const*/ char *p, /*const*/ char *ep) {
  switch (*p) {
    case '.': return 1;  /* matches any char */
    case REGEXP_ESC: return match_class(c, uchar(*(p+1)));
    case '[': return matchbracketclass(c, p, ep-1);
    default:  return (uchar(*p) == c);
  }
} // end of singlematch


//static const
char *match (MatchState *ms, /*const*/ char *s, /*const*/ char *p);


//static const
char *matchbalance (MatchState *ms, /*const*/ char *s,
                                 /*const*/ char *p) {
  if (*p == 0 || *(p+1) == 0)
    error(ERR_UNBALANCED_PATTERN);
  if (*s != *p) return NULL;
  else {
    int b = *p;
    int e = *(p+1);
    int cont = 1;
    while (++s < ms->src_end) {
      if (*s == e) {
        if (--cont == 0) return s+1;
      }
      else if (*s == b) cont++;
    }
  }
  return NULL;  /* string ends out of balance */
} //  end of matchbalance


//static const
char *max_expand (MatchState *ms, /*const*/ char *s,
                               /*const*/ char *p, /*const*/ char *ep) {
  int i = 0;  /* counts maximum expand for item */
  while ((s+i)<ms->src_end && singlematch(uchar(*(s+i)), p, ep))
    i++;
  /* keeps trying to match with the maximum repetitions */
  while (i>=0) {
    //const
    char *res = match(ms, (s+i), ep+1);
    if (res) return res;
    i--;  /* else didn't match; reduce 1 repetition to try again */
  }
  return NULL;
} // end of max_expand


//static const
char *min_expand (MatchState *ms, /*const*/ char *s,
                               /*const*/ char *p, /*const*/ char *ep) {
  for (;;) {
    //const
    char *res = match(ms, s, ep+1);
    if (res != NULL)
      return res;
    else if (s<ms->src_end && singlematch(uchar(*s), p, ep))
      s++;  /* try with one more repetition */
    else return NULL;
  }
} // end of min_expand


//static const
char *start_capture (MatchState *ms, /*const*/ char *s,
                                  /*const*/ char *p, int what) {
  //const
  char *res;
  int level = ms->level;
  if (level >= MAXCAPTURES) error(ERR_TOO_MANY_CAPTURES);
  ms->capture[level].init = s;
  ms->capture[level].len = what;
  ms->level = level+1;
  if ((res=match(ms, s, p)) == NULL)  /* match failed? */
    ms->level--;  /* undo capture */
  return res;
} // end of start_capture


//static const
char *end_capture (MatchState *ms, /*const*/ char *s,
                                /*const*/ char *p) {
  int l = capture_to_close(ms);
  //const
  char *res;
  ms->capture[l].len = s - ms->capture[l].init;  /* close capture */
  if ((res = match(ms, s, p)) == NULL)  /* match failed? */
    ms->capture[l].len = CAP_UNFINISHED;  /* undo capture */
  return res;
} // end of end_capture


//static const
char *match_capture (MatchState *ms, /*const*/ char *s, int l) {
  size_t len;
  l = check_capture(ms, l);
  len = ms->capture[l].len;
  if ((size_t)(ms->src_end-s) >= len &&
      memcmp(ms->capture[l].init, s, len) == 0)
    return s+len;
  else return NULL;
} // end of match_capture


//static const
char *match (MatchState *ms, /*const*/ char *s, /*const*/ char *p) {
init: /* using goto's to optimize tail recursion */
  switch (*p) {
    case '(': {  /* start capture */
      if (*(p+1) == ')')  /* position capture? */
        return start_capture(ms, s, p+2, CAP_POSITION);
      else
        return start_capture(ms, s, p+1, CAP_UNFINISHED);
    }
    case ')': {  /* end capture */
      return end_capture(ms, s, p+1);
    }
    case REGEXP_ESC: {
      switch (*(p+1)) {
        case 'b': {  /* balanced string? */
          s = matchbalance(ms, s, p+2);
          if (s == NULL) return NULL;
          p+=4; goto init;  /* else return match(ms, s, p+4); */
        }
        case 'f': {  /* frontier? */
          //const
          char *ep; char previous;
          p += 2;
          if (*p != '[')
            error(ERR_MISSING_LH_SQUARE_BRACKET_AFTER_ESC_F);
          ep = classend(ms, p);  /* points to what is next */
          previous = (s == ms->src) ? '\0' : *(s-1);
          if (matchbracketclass(uchar(previous), p, ep-1) ||
              !matchbracketclass(uchar(*s), p, ep-1)) return NULL;
          p=ep; goto init;  /* else return match(ms, s, ep); */
        }
        default: {
          if (isdigit(uchar(*(p+1)))) {  /* capture results (%0-%9)? */
            s = match_capture(ms, s, uchar(*(p+1)));
            if (s == NULL) return NULL;
            p+=2; goto init;  /* else return match(ms, s, p+2) */
          }
          goto dflt;  /* case default */
        }
      }
    }
    case '\0': {  /* end of pattern */
      return s;  /* match succeeded */
    }
    case '$': {
      if (*(p+1) == '\0')  /* is the `$' the last char in pattern? */
        return (s == ms->src_end) ? s : NULL;  /* check end of string */
      else goto dflt;
    }
    default: dflt: {  /* it is a pattern item */
      //const
      char *ep = classend(ms, p);  /* points to what is next */
      int m = s<ms->src_end && singlematch(uchar(*s), p, ep);
      switch (*ep) {
        case '?': {  /* optional */
          //const
          char *res;
          if (m && ((res=match(ms, s+1, ep+1)) != NULL))
            return res;
          p=ep+1; goto init;  /* else return match(ms, s, ep+1); */
        }
        case '*': {  /* 0 or more repetitions */
          return max_expand(ms, s, p, ep);
        }
        case '+': {  /* 1 or more repetitions */
          return (m ? max_expand(ms, s+1, p, ep) : NULL);
        }
        case '-': {  /* 0 or more repetitions (minimum) */
          return min_expand(ms, s, p, ep);
        }
        default: {
          if (!m) return NULL;
          s++; p=ep; goto init;  /* else return match(ms, s+1, ep); */
        }
      }
    }
  }
} // end of match


// functions below written by Nick Gammon ...

char MatchState::Match (/*const*/ char * pattern, unsigned int index)
{
  // set up for throwing errors
  char rtn = setjmp (regexp_error_return);

  // error return
  if (rtn)
    return ((result = rtn));

  if (!src)
    error (ERR_NO_TARGET_STRING);

  if (index > src_len)
    index = src_len;

  int anchor = (*pattern == '^') ? (pattern++, 1) : 0;
  //const
  char *s1 =src + index;
  src_end = src + src_len;

  // iterate through target string, character by character unless anchored
  do {
    //const
    char *res;
    level = 0;
    if ((res=match(this, s1, pattern)) != NULL)
    {
      MatchStart = s1 - src;
      MatchLength = res - s1;
      return (result = REGEXP_MATCHED);
    }  // end of match at this position
  } while (s1++ < src_end && !anchor);

  return (result = REGEXP_NOMATCH); // no match

} // end of regexp

// set up the target string
void MatchState::Target (char * s)
{
  Target (s, strlen (s));
}  // end of MatchState::Target

void MatchState::Target (char * s, /*const*/ unsigned int len)
{
  src = s;
  src_len = len;
  result = REGEXP_NOMATCH;
}  // end of MatchState::Target

// copy the match string to user-supplied buffer
// buffer must be large enough to hold it
char * MatchState::GetMatch (char * s) const
{
  if (result != REGEXP_MATCHED)
    s [0] = 0;
  else
    {
    memcpy (s, &src [MatchStart], MatchLength);
    s [MatchLength] = 0;  // null-terminated string
    }
  return s;
} // end of  MatchState::GetMatch

// get one of the capture strings (zero-relative level)
// buffer must be large enough to hold it
char * MatchState::GetCapture (char * s, /*const*/ int n) const
{
  if (result != REGEXP_MATCHED || n >= level || capture [n].len <= 0)
    s [0] = 0;
  else
    {
    memcpy (s, capture [n].init, capture [n].len);
    s [capture [n].len] = 0;  // null-terminated string
    }
  return s;
} // end of MatchState::GetCapture

// match repeatedly on a string, return count of matches
unsigned int MatchState::MatchCount (/*const*/ char * pattern)
{
  unsigned int count = 0;

  // keep matching until we run out of matches
  for (unsigned int index = 0;
       Match (pattern, index) > 0 &&
       index < src_len;                       // otherwise empty matches loop
       count++)
    // increment index ready for next time, go forwards at least one byte
    index = MatchStart + (MatchLength == 0 ? 1 : MatchLength);

  return count;

} // end of MatchState::MatchCount

// match repeatedly on a string, call function f for each match
unsigned int MatchState::GlobalMatch (/*const*/ char * pattern, GlobalMatchCallback f)
{
  unsigned int count = 0;

  // keep matching until we run out of matches
  for (unsigned int index = 0;
       Match (pattern, index) > 0;
       count++)
    {
    f (& src [MatchStart], MatchLength, *this);
    // increment index ready for next time, go forwards at least one byte
    index = MatchStart + (MatchLength == 0 ? 1 : MatchLength);
    } // end of for each match
  return count;

} // end of MatchState::GlobalMatch

// match repeatedly on a string, call function f for each match
//  f sets replacement string, incorporate replacement and continue
// maximum of max_count replacements if max_count > 0
// replacement string in GlobalReplaceCallback must stay in scope (eg. static string or literal)
unsigned int MatchState::GlobalReplace (/*const*/ char * pattern, GlobalReplaceCallback f, /*const*/ unsigned int max_count)
{
  unsigned int count = 0;

  // keep matching until we run out of matches
  for (unsigned int index = 0;
       Match (pattern, index) > 0 &&            // stop when no match
       index < src_len &&                       // otherwise empty matches loop
       (max_count == 0 || count < max_count);   // stop when count reached
       count++)
    {
    // default is to replace with self
    //const
    char * replacement = &src [MatchStart];
    unsigned int replacement_length = MatchLength;

    // increment index ready for next time, go forwards at least one byte
    if (MatchLength == 0)
      index = MatchStart + 1; // go forwards at least one byte or we will loop forever
    else
      {
      // increment index ready for next time,
      index = MatchStart + MatchLength;

      // call function to find replacement text
      f (&src [MatchStart], MatchLength, replacement, replacement_length, *this);

      // see how much memory we need to move
      int lengthDiff = MatchLength - replacement_length;

      // copy the rest of the buffer backwards/forwards to allow for the length difference
      memmove (&src [index - lengthDiff], &src [index], src_len - index);

      // copy in the replacement
      memmove (&src [MatchStart], replacement, replacement_length);

      // adjust the index for the next search
      index -= lengthDiff;
      // and the length of the source
      src_len -= lengthDiff;
      } // end if matching at least one byte
    } // end of for each match

  // put a terminating null in
  src [src_len] = 0;
  return count;
} // end of MatchState::GlobalReplace


// match repeatedly on a string, replaces with replacement string for each match
// maximum of max_count replacements if max_count > 0
// replacement string in GlobalReplaceCallback must stay in scope (eg. static string or literal)
unsigned int MatchState::GlobalReplace (/*const*/ char * pattern, /*const*/ char * replacement, /*const*/ unsigned int max_count)
{
  unsigned int count = 0;
  unsigned int replacement_length = strlen (replacement);

  // keep matching until we run out of matches
  for (unsigned int index = 0;
       Match (pattern, index) > 0 &&           // stop when no match
       index < src_len &&                      // otherwise empty matches loop
       (max_count == 0 || count < max_count);  // stop when count reached
       count++)
    {
    if (MatchLength == 0)
      index = MatchStart + 1; // go forwards at least one byte or we will loop forever
    else
      {
      // increment index ready for next time,
      index = MatchStart + MatchLength;

      // see how much memory we need to move
      int lengthDiff = MatchLength - replacement_length;

      // copy the rest of the buffer backwards/forwards to allow for the length difference
      memmove (&src [index - lengthDiff], &src [index], src_len - index);

      // copy in the replacement
      memmove (&src [MatchStart], replacement, replacement_length);

      // adjust the index for the next search
      index -= lengthDiff;
      // and the length of the source
      src_len -= lengthDiff;
      } // end if matching at least one byte

    } // end of for each match

  // put a terminating null in
  src [src_len] = 0;
  return count;
} // end of MatchState::GlobalReplace