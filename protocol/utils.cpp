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

#include "utils.h"
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

bool write_to_fd(int fd, char const * buf, size_t count)
{
  ssize_t length_to_write = count;
  ssize_t bytes_written = 0;
  while(length_to_write>0)
  {
    ssize_t res = write(fd, buf + bytes_written, length_to_write);
    if (res < 0)
    {
      if (errno == EINTR)
        continue;
      return false;
    }
    bytes_written += res;
    length_to_write -= res;
  }
  return true;
}


bool write_to_fd(int fd, const std::string &a)
{
  return write_to_fd(fd, a.c_str(), a.length());
  return true;
}

bool write_to_file(const std::string &fname, const std::string &a)
{
  int fd = open(fname.c_str(), O_CREAT | O_CLOEXEC | O_WRONLY);
  if (fd < 0)
    return false;
  bool res = write_to_fd(fd, a);

  if (close(fd)!=0)
    return false;
  return res;
}

void Split(const std::string &what, std::string &l, std::string &r, const std::string &delimiter)
{
  std::string::size_type p;
  p = what.find(delimiter);
  if (p==std::string::npos)
  {
    r = "";
    l = what;
  }else
  {
    l = what.substr(0, p);
    r = what.substr(p+delimiter.length());
  }
}



void Trim(std::string &s, const std::string &trims = "\t\n ")
{
  size_t startpos = s.find_first_not_of(trims.c_str());
  size_t endpos = s.find_last_not_of(trims.c_str());
  if(( std::string::npos == startpos ) || (std::string::npos == endpos))
  {
    s = "";
  }
  else
    s = s.substr(startpos, endpos-startpos +1);
}

std::string ParseEnum(const std::string &list, int value)
{
  std::string str = list;
  std::map<int, std::string> strings;    
  int cur_val = -1;
  do{
    std::string l, r;
    Split(str,l,r,",");
    // std::cout << "Left: " << l << std::endl;
    // std::cout << "Right: " << r << std::endl;
    str=r;
    std::string &what = l;
    {
      std::string l, r;
      Split(what,l, r, "=");
      Trim(r);
      Trim(l);
      if (r!="")
  cur_val=Convert<int>(r);
      else
  cur_val++;
      //      std::cout << "adding " << l << " " << cur_val << " " << r << std::endl;
      strings[cur_val]=l;
    }
  }while(str!="");
  return strings[value];
} 

void ThrowError(const std::string msg)
{
  const int buf_size=256;
  char buf[buf_size];
  char * err =  strerror_r(errno, buf, buf_size);
  throw std::runtime_error(msg +": " + err);  
}

std::string UnPack(const unsigned char* pabSource, int nSrcSize)
{
  int i;
  unsigned char b;

  if(nSrcSize <= 0)
    throw(std::runtime_error( "UnPack: bad nSrcSize!"));

  nSrcSize *= 2;
  std::string str;
  for(i = 0; i < nSrcSize; i++)
  {
    if(i & 0x01)
      b = pabSource[i / 2] & 0x0f;
    else
      b = (pabSource[i / 2] & 0xf0) >> 4;

    str += (unsigned char) (b > 9) ? (b + 0x37) : (b + 0x30);
  }
  return str;
}

void Pack(unsigned char* pabDest,const unsigned char* strSource, int32_t* pnLen)
{
  int32_t nSize, i;
  int b;

  if( (nSize = strlen((char *)strSource)) <= 0)
    throw(std::runtime_error( "Pack: bad strSource!"));

  for(i = 0; i < nSize; i++)
  {
    b = toupper(strSource[i]);
    b = (b > 0x39) ? (b - 0x37) : (b - 0x30);

    if((b < 0) || (b > 0x0f))
      throw(std::runtime_error(  "Pack: Source string contains no ascii symbols!"));

    if(!(i & 0x01))
    {
      b <<= 4;
      pabDest[i / 2] = b&0xff;
    }
    else
      pabDest[i / 2] |= b&0xff;
  }
  if(pnLen)
  {
    if(nSize & 0x01)
      nSize++;
    (*pnLen) = nSize / 2;
  }
}

std::string BinToASCII(const std::vector<unsigned char>& chars)
{
  return UnPack((unsigned char *)&chars[0], chars.size());
}

std::vector<unsigned char> ASCIIToBin(const std::string& input)
{
  size_t retsize = (input.size() / 2) + ((input.size() & 0x01) ? 1 : 0) ;
  std::vector<unsigned char> ret(retsize);
  Pack((unsigned char *)&ret[0], (unsigned char *)input.c_str());
  return ret;
}
// Function to convert decimal number to octal
int DecToOct(int nDec)
{
  int nRem, i = 1, nOct = 0;
  while(nDec != 0)
  {
    nRem = nDec % 8;
    nDec /= 8;
    nOct += nRem * i;
    i *= 10;
  }
  return nOct;
}
