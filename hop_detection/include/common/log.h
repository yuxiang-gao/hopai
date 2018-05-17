#ifndef MY_COMMON_LOG_H_
#define MY_COMMON_LOG_H_

#define NOTICE(text, flag) {               \
  //static bool flag = true;           
  if(flag) {                         \
    std::cout << text << std::endl;  \
    flag = false;                    \
  }                                  \
}                                    \

#endif