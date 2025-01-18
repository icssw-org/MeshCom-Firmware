Import("env")
import os
import sys

env.Replace(PROGNAME="%s" % env.GetProjectOption("custom_filename"))