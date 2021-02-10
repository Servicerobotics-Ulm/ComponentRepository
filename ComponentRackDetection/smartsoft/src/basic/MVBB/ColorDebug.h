#ifndef _COLORDEBUG_H_
#define _COLORDEBUG_H_

#include <stdio.h>
#include <string>
#include <string.h>
#include <sstream>
#include <iomanip>
#include <stdlib.h>

using namespace std;

namespace
{
  enum COLOR
  {
    BLACK = 30,
    RED = 31,
    GREEN = 32,
    YELLOW = 33,
    BLUE = 34,
    MAGENTA = 35,
    CYAN = 36,
    WHITE = 37,
    BG_BLACK = 40,
  };

  /// A helper class to enable colored debug output
  class ColorDebug
  {
  public:

    static bool verbose;
    static pair<bool, bool> tabBoldness;
    static unsigned int badgerTab;

    /**
     * Get a colored text by using char codes.
     * @param c The (foreground) color for the text. Valid colors are
     * between 30 and 36, see enum in globals.h also. If c is not in this
     * interval, color will be black.
     * @param text The text that shall be colored. If given, color is set
     * to black again after the string. Otherwise, color is kept.
     * @return The colored array of chars (can be used for printf or cout).
     */
    static const char* color(int c, const char* text="", bool bold=false)
    {
      string tmpstr;
      char _tmpcol[128];
      memset(_tmpcol, 0, 128*sizeof(char));

      if (c < 30 || c > 37)
      {
	if (c == 40) sprintf(_tmpcol, "%c[%d;%d;%dm", 0x1B, bold, c, 40);
	else sprintf(_tmpcol, "%c[%dm", 0x1B, bold);
      }
      else if (strcmp(text,"")==0) sprintf(_tmpcol, "%c[%d;%dm", 0x1B, bold, c);
      else sprintf(_tmpcol, "%c[%d;%dm%s%c[%dm", 0x1B, bold, c, text, 0x1B, 0);
      tmpstr = _tmpcol;
      return tmpstr.c_str();
    };

    static const char* bcolor(int c, const char* text="")
    {
      return color(c, text, true);
    };

    static void BADGR_LOGO(unsigned int index = 0)
    {
      index = (index > 23 ? 23 : index);
      stringstream s;
      if (index < 1) s << endl;
      if (index < 1 || index == 2)
      {
	s << "   " << ColorDebug::bcolor(CYAN);
	s << "@" << ColorDebug::color(0);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@" << ColorDebug::color(CYAN);
	s << "@ @@" << ColorDebug::color(0);
	s << "@@" << ColorDebug::bcolor(BLACK);
	s << "@   " << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::color(0);
	s << "@@" << ColorDebug::bcolor(BLACK);
	s << "@" << ColorDebug::color(CYAN);
	s << "@   ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 3)
      {
	s << "  " << ColorDebug::bcolor(BLACK);
	s << "@@@ @" << ColorDebug::color(0);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@" << ColorDebug::bcolor(CYAN);
	s << "@" << ColorDebug::color(0);
	s << "@ " << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::color(0);
	s << "@@" << ColorDebug::bcolor(BLACK);
	s << "@@@" << ColorDebug::bcolor(CYAN);
	s << "@@@  ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 4)
      {
	s << "   " << ColorDebug::color(0);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@@@" << ColorDebug::color(CYAN);
	s << "@    " << ColorDebug::bcolor(BLACK);
	s << "@@@" << ColorDebug::color(0);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@" << ColorDebug::color(0);
	s << "@@@  ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 5)
      {
	s << "   " << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@@@    @@@@@@@" << ColorDebug::color(0);
	s << "@@  ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 6)
      {
	s << "    " << ColorDebug::color(0);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@@    @@@@@@@@" << ColorDebug::color(0);
	s << "@  ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 7)
      {
	s << "   " << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@@" << ColorDebug::bcolor(CYAN);
	s << "@    " << ColorDebug::bcolor(BLACK);
	s << "@@@@@@@" << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::color(0);
	s << "@" << ColorDebug::color(CYAN);
	s << "@ ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 8)
      {
	s << "   " << ColorDebug::bcolor(CYAN);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@@     @@@@@@" << ColorDebug::color(CYAN);
	s << "@ " << ColorDebug::color(0);
	s << "@  ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 9)
      {
	s << "  " << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::bcolor(CYAN);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@@     @@@@@" << ColorDebug::color(CYAN);
	s << "@  " << ColorDebug::bcolor(CYAN);
	s << "@" << ColorDebug::color(CYAN);
	s << "@ ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 10)
      {
	s << "  " << ColorDebug::color(CYAN);
	s << "@@" << ColorDebug::bcolor(CYAN);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@" << ColorDebug::bcolor(CYAN);
	s << "@     " << ColorDebug::bcolor(BLACK);
	s << "@@@@@" << ColorDebug::color(0);
	s << "   @  ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 11)
      {
	s << "  " << ColorDebug::color(0);
	s << "@" << ColorDebug::bcolor(CYAN);
	s << "@" << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@     @@@@" << ColorDebug::color(CYAN);
	s << "@  " << ColorDebug::bcolor(CYAN);
	s << "@" << ColorDebug::color(CYAN);
	s << "@  ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 12)
      {
	s << "   " << ColorDebug::color(0);
	s << "@" << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@    " << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@@   " << ColorDebug::color(0);
	s << "@   ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 13)
      {
	s << "   " << ColorDebug::color(0);
	s << "@" << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::color(0);
	s << "<" << ColorDebug::bcolor(BLACK);
	s << "o>    @<o" << ColorDebug::color(0);
	s << ">" << ColorDebug::bcolor(BLACK);
	s << "@   " << ColorDebug::color(0);
	s << "@   ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 14)
      {
	s << "   " << ColorDebug::color(CYAN);
	s << "@@" << ColorDebug::bcolor(BLACK);
	s << "@@@" << ColorDebug::color(CYAN);
	s << "    " << ColorDebug::bcolor(BLACK);
	s << "@@@@" << ColorDebug::bcolor(CYAN);
	s << "@  " << ColorDebug::color(0);
	s << "@    ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 15)
      {
	s << "    " << ColorDebug::color(0);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@" << ColorDebug::color(CYAN);
	s << "    " << ColorDebug::color(BLACK);
	s << "@@@@" << ColorDebug::color(CYAN);
	s << "  " << ColorDebug::color(BLACK);
	s << "@" << ColorDebug::color(CYAN);
	s << "@    ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 16)
      {
	s << "    " << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@" << ColorDebug::color(CYAN);
	s << "    " << ColorDebug::bcolor(BLACK);
	s << "@@@" << ColorDebug::color(CYAN);
	s << "  @" << ColorDebug::bcolor(BLACK);
	s << "@    ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 17)
      {
	s << "     " << ColorDebug::color(0);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@" << ColorDebug::color(CYAN);
	s << "@    " << ColorDebug::bcolor(BLACK);
	s << "@@@" << ColorDebug::color(CYAN);
	s << " @" << ColorDebug::color(0);
	s << "@      ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 18)
      {
	s << "     " << ColorDebug::color(0);
	s << "@ " << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::color(0);
	s << "@@" << ColorDebug::color(CYAN);
	s << "@ " << ColorDebug::color(BLACK);
	s << "@@" << ColorDebug::bcolor(CYAN);
	s << "@ " << ColorDebug::color(0);
	s << "@       ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 19)
      {
	s << "     " << ColorDebug::bcolor(CYAN);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@" << ColorDebug::color(0);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@" << ColorDebug::color(CYAN);
	s << "@ " << ColorDebug::bcolor(BLACK);
	s << "@" << ColorDebug::color(0);
	s << " @" << ColorDebug::color(CYAN);
	s << "@       ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 20)
      {
	s << "      " << ColorDebug::color(0);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@@@" << ColorDebug::color(0);
	s << "@ " << ColorDebug::color(CYAN);
	s << "@" << ColorDebug::color(0);
	s << "@";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 21)
      {
	s << "      " << ColorDebug::color(0);
	s << "@@" << ColorDebug::bcolor(BLACK);
	s << "@" << ColorDebug::color(0);
	s << "@@@@@          ";
	if (index < 1) s << endl;
      }
      if (index < 1 || index == 22)
      {
	s << "      " << ColorDebug::color(0);
	s << "@" << ColorDebug::bcolor(BLACK);
	s << "@@" << ColorDebug::color(0);
	s << "@@" << ColorDebug::color(CYAN);
	s << "@         ";
	if (index < 1) s << endl;
      }
      if (index < 1)
	s << endl;
      if (index > 22)
	ColorDebug::badgerTab = 0;

      cout << s.str().c_str();
    };

    static const void tabPlot(const char *txt1, const char *txt2, int c1=0, int c2=0, int l1=25, int l2=25)
    {
      cout<<txt1<<txt2<<std::endl;
      return; //TODO The lower code enforces an invalid read MATTHIAS
      if (!verbose) return;

      cout << (ColorDebug::tabBoldness.first ? bcolor(c1) : color(c1));
      cout << left << setw(l1) << setfill('.') << txt1 << " ";
      cout << (ColorDebug::tabBoldness.second ? bcolor(c2) : color(c2));
      cout << left << setw(l2) << setfill(' ') << txt2;

      if (ColorDebug::badgerTab > 0)
      {
	cout << left << setw(25);
	ColorDebug::BADGR_LOGO(ColorDebug::badgerTab+=2);
      }

      cout << color(0) << endl;
    };

    static const void btabPlot(const char *txt1, const char *txt2, int c1=0, int c2=0, int l1=25, int l2=25)
    {
      pair<bool,bool> old = ColorDebug::tabBoldness;
      ColorDebug::setTabBoldness(true, true);
      tabPlot(txt1, txt2, c1, c2, l1, l2);
      ColorDebug::setTabBoldness(old.first, old.second);
    };

    static const void tabPlot(const char *txt1, stringstream &txt2, int c1=0, int c2=0, int l1=25, int l2=25)
    {
      tabPlot(txt1, txt2.str().c_str(), c1, c2, l1, l2);
    };

    static const void btabPlot(const char *txt1, stringstream &txt2, int c1=0, int c2=0, int l1=25, int l2=25)
    {
      btabPlot(txt1, txt2.str().c_str(), c1, c2, l1, l2);
    };

    static void setTabBoldness(bool left, bool right)
    {
      ColorDebug::tabBoldness = pair<bool,bool>(left, right);
    };

    /**
     * A helper function to ease debug logfile dumping.
     * @param filename Simple name of what this logfile is for
     * @return A string to the conveniently save a file in Data/log/[...].log
     */
    static const char* logfile(const char *filename, std::string s="")
    {
      char l[256];
      sprintf(l, "%s/Data/log/%s.log", getenv("BOXGRASPINGSUITE"), filename);
      s=l; return s.c_str();
    };

    static void badgerStart()
    {
      ColorDebug::badgerTab = 1;
    };

    static void badgerFinish()
    {
      while (ColorDebug::badgerTab != 0)
      {
	ColorDebug::BADGR_LOGO(ColorDebug::badgerTab+=2);
	cout << endl;
      }
    };
  };

  unsigned int ColorDebug::badgerTab = 0;
  bool ColorDebug::verbose = true;
  pair<bool,bool> ColorDebug::tabBoldness = pair<bool,bool>(false, false);
}

#endif // _COLORDEBUG_H_
