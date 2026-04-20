#pragma once

/**
 * @file WZ_dbLOG.h
 * @author W.Zelinka (OE3WAS, https://github.com/karamo)
 * @brief 
 * @version 0.1
 * @date 2026-04-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */
 void dbLOG(const char * fmt, ...);
 void dbCLOG(bool condition, const char * fmt, ...);
 void dbLLOG(int DBGVAR, int Level, const char * fmt, ...);
