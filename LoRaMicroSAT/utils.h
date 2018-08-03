/*
 * Copyright (C) 2018  Ivan Stoev
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
 */
#ifndef __UTILS_H_INCLUDED__
#define __UTILS_H_INCLUDED__

#include <string>

#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

void Split(const std::string& what, std::string& l, std::string& r, const std::string& delimiter);
void Trim(std::string& s, const std::string& trims);
std::string ParseEnum(const std::string& list, int value);
void ThrowError(const std::string msg);

template <typename T, typename V>
inline T Convert(const V& str)
{
    T var;
    std::stringstream iss;
    iss << str;
    iss >> var;
    // deal with any error bits that may have been set on the stream
    return var;
}
bool write_to_fd(int fd, const std::string &a);
bool write_to_fd(int fd, char const * buf, size_t count);

bool write_to_file(const std::string &fname, const std::string &a);

template <class T>
inline bool write_to_file(const std::string &fname, const T &a)
{
  return write_to_file(fname, Convert<std::string>(a));
}


template <typename T>
inline T Convert(const std::string& str)
{
    T var;
    std::istringstream iss;
    iss.str(str);
    if (str.find("0x")!=std::string::npos)
      iss>>std::hex;
    iss >> var;
    // deal with any error bits that may have been set on the stream
    if (iss)
      return var;
    else
      return T();
}



#define MAKE_ENUM(name, ...) enum  name { __VA_ARGS__}; \
inline std::ostream& operator<<(std::ostream& os, name value) {  \
  os << #name << "::" << ParseEnum(#__VA_ARGS__, value);  \
  return os;              \
}

template<class T>
inline void HexDump(const T&buf)
{
  for(int i = 0; i < buf.size(); ++i)
  {
    std::cout.fill('0');
    std::cout << std::hex << std::setw(2) << (int)(unsigned char)buf[i] << " ";
  }
  std::cout << std::dec << std::endl;
}

std::string UnPack(const unsigned char* pabSource, int nSrcSize);
void Pack(unsigned char* pabDest,const unsigned char* strSource, int32_t* pnLen = NULL);
std::string BinToASCII(const std::vector<unsigned char>& chars);
std::vector<unsigned char> ASCIIToBin(const std::string& input);

#endif
