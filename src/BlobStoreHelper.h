#ifndef BLOBHELPER_H
#define BLOBHELPER_H

#ifndef Q_MOC_RUN

#include <iostream>
#include <curl/curl.h>
#include <QFile>
#include <QUrl>
#include <QTextStream>
#include <regex>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <functional>
#include <cstring>
#endif

using std::regex;
using std::string;
using std::sregex_token_iterator;

namespace blobhelper{

struct MemoryStruct {
  char *memory;
  size_t size;
};

extern "C" size_t write_data(void *contents, size_t size, size_t nmemb, void *userp);

class BlobStoreHelper
{
public:

    static bool addBlob(const std::string file_full_path, const std::string url);

    static bool getBlob(const std::string url, std::string& data);

    static bool deleteBlob(const std::string url);

private:


};
};
#endif // BLOBHELPER_H
