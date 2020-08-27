#ifndef IO_UTILS_H
#define IO_UTILS_H

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
 #include <conio.h>   //for getch(), needed in wait_for_key()
#endif

#include <iostream>
#include <fstream>
#include <memory>
#include <iomanip>
#include <cerrno>
#include <streambuf>

#include <unistd.h> //used for custon getch()
#include <termios.h> //used for custon getch()
#include <fcntl.h>

/** \brief Pauses until a key is pressed.
 */
inline void wait_for_key()
{
  #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)  // every keypress registered, also arrow keys
      std::cout << std::endl << "Press any key to continue..." << std::endl;

      FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
      _getch();
  #elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
      std::cout << std::endl << "Press ENTER to continue..." << std::endl;

      std::cin.clear();
      std::cin.ignore(std::cin.rdbuf()->in_avail());
      std::cin.get();
  #endif
      return;
}


/** \brief Returns 1 if a key on the keyboard is pressed, 0 otherwise.
 *  @return 1 if a key on the keyboard is pressed, 0 otherwise.
 */
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}


/** \brief Reads a character from the keyboard (no need to press 'enter' afterwards is required)
 *  @return the character pressed in the keyboard.
 */
char getch()
{
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
            perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror ("tcsetattr ~ICANON");
    return (buf);
}

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[34m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[32m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[33m" << "[WARNING]: " << msg << "\033[0m";
}

void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[31m" << "[ERROR]: " << msg << "\033[0m";
}

#endif // IO_UTILS_H
