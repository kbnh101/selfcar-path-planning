#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <istream>

using namespace std;


vector<string> csv_read_row(istream& file, char delimiter);

vector<string> csv_read_row(istream& file, char delimiter)
{
  stringstream ss;
  bool inquotes = false;
  vector<string> row;
  while (file.good())
  {
      char c = file.get();
      if (!inquotes && c == '"')
      {
          inquotes = true;
      }
      else if (inquotes && c == '"')
      {
          if (file.peek() == '"')
          {
              ss << (char)file.get();
          }
          else
          {
              inquotes = false;
          }
      }
      else if (!inquotes && c == delimiter)
      {
          row.push_back(ss.str());
          ss.str("");
      }
      else if (!inquotes && (c == '\r' || c == '\n'))
      {
          if (file.peek() == '\n') { file.get(); }
          row.push_back(ss.str());
          return row;
      }
      else
      {
          ss << c;
      }
  }
}
