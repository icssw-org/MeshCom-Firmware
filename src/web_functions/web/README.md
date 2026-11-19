# Web GUI assets

The CSS and JavaScript of the node's web GUI live here as plain files:

| File      | Content                | Boards (with web GUI) |
| --------- | ---------------------- | --------------------- |
| `mc.css`  | styles of every page   | all                   |
| `mc.js`   | shared page logic      | all                   |
| `mcau.js` | Auto Update card       | ESP32 only            |
| `mcrm.js` | Remote management page | all                   |

The firmware does not read these files. It serves gzip copies that are compiled into
`src/web_functions/web_assets.h`.

## After changing a file here

Regenerate the header and commit both:

```bash
python3 tools/gen_web_assets.py
```

Check whether the header matches the sources (exit code 1 when it is out of date):

```bash
python3 tools/gen_web_assets.py --check
```

The script needs only Python 3 with its standard library. It drops blank lines, full-line `//` comments in JS and
one-line `/* ... */` comments in CSS, compresses
each file with gzip and writes the cache tag (`?v=`) from a hash of the stripped text.

## Rules

- **Never edit `web_assets.h` by hand.** The next run of the script overwrites it.
- **Commit the regenerated header.** The build compiles the header as it is in the repo; it does not run the script.
  A forgotten run means the node serves the old CSS or JS.
- **Comments are almost free.** Full-line `//` comments (JS) and one-line `/* ... */` comments (CSS) do not reach
  the node. Multi-line CSS comments and comments after code do.
