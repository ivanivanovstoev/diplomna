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

#pragma once
#include <sstream>
#ifndef FUNC
#if defined(_WIN32) || defined(_WIN64)
#define FUNC __FUNCTION__
#else
#define FUNC __PRETTY_FUNCTION__
#endif
#endif

#define  LOG_EMERG  0  /* system is unusable */
#define  LOG_ALERT  1  /* action must be taken immediately */
#define  LOG_CRIT  2  /* critical conditions */
#define  LOG_ERR    3  /* error conditions */
#define  LOG_WARNING  4  /* warning conditions */
#define  LOG_NOTICE  5  /* normal but significant condition */
#define  LOG_INFO  6  /* informational */
#define  LOG_DEBUG  7  /* debug-level messages */

template<class ErrorClass>
class IostreamLogger
{
  ErrorClass m_log_type;
  std::ostringstream m_os;
  std::string m_context;
  bool m_moved;
public:
  IostreamLogger(ErrorClass elog, const std::string &ctx):
    m_log_type(elog),
    m_context(ctx),
    m_moved(false)
  {
  }
  IostreamLogger(IostreamLogger &&other):
    m_log_type(std::move(other.m_log_type)),
    m_os(other.m_os.str()),
    m_context(std::move(other.m_context)),
    m_moved(false)
  {
    other.m_moved = true;
  }
  template<class T>
  IostreamLogger &operator << (const T &a)
  {
    m_os << a;
    return *this;
  }
  ~IostreamLogger(){
    if(! m_moved)
      std::cout <<m_context.c_str() << " "<< m_os.str().c_str() << std::endl;
  }
};

template<class T>
IostreamLogger<T> IostreamLog(T t,  const std::string &ctx)
{
  return IostreamLogger<T>(t, ctx);
}


struct NoLogger
{
  template<class T>
  NoLogger &operator << (const T &a)
  {
    return *this;
  }
};
inline NoLogger NoLog()
{
  return NoLogger();
}

static const int LOG_LAST=0x172343;// we hope it's different than all of the other constants
inline int GetType(int type)
{
  thread_local  static  int last_type = LOG_NOTICE;
  if (type != LOG_LAST)
    last_type = type;
  return last_type;
}

#define ILOG_MSG(context) IostreamLog(GetType(LOG_NOTICE), context)
#define ILOG_ERR(context) IostreamLog(GetType(LOG_ERR), context)
#define ILOG_WAR(context) IostreamLog(GetType(LOG_WARNING), context)
#define ILOG_DBG(context) IostreamLog(GetType(LOG_DEBUG), context)
#define ILOG_LST(context) IostreamLog(GetType(LOG_LAST), context)


#define IMSG ILOG_MSG(FUNC)
#define IWAR ILOG_WAR(FUNC)
#define IERR ILOG_ERR(FUNC)
#define ILST ILOG_LST(FUNC)
#define IDBG ILOG_DBG(FUNC)

#define ILOG IMSG
