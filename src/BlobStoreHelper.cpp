#include "BlobStoreHelper.h"

namespace blobhelper{


bool BlobStoreHelper::addBlob(const std::string file_full_path, const std::string url){

    std::cout << "add blob ----> " << QUrl(url.c_str()).toLocalFile().toStdString() << "to location " << file_full_path << std::endl;
    return QFile::copy(QString(file_full_path.c_str()), QUrl(url.c_str()).toLocalFile());
}


size_t write_data(void *contents, size_t size, size_t nmemb, void *userp)
{
    size_t realsize = size * nmemb;
    //    printf("size --> %d\n", (int)size);
    //    printf("nmemb --> %d\n", (int)nmemb);
    //    printf("realsize --> %d\n", (int)realsize);

    struct MemoryStruct *mem = (struct MemoryStruct *)userp;

    mem->memory = (char *)realloc(mem->memory, mem->size + realsize + 1);
    if(mem->memory == NULL) {
        /* out of memory! */
        printf("not enough memory (realloc returned NULL)\n");
        return 0;
    }

    memcpy(&(mem->memory[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->memory[mem->size] = 0;

    return realsize;
}

bool BlobStoreHelper::getBlob(const std::string url, std::__cxx11::string &data)
{
    std::cout << "getting blob ---> " << url << std::endl;

    std::cout << "local file --> " << QUrl(url.c_str()).toLocalFile().toStdString() << std::endl;

    QFile file(QUrl(url.c_str()).toLocalFile());

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        std::cout << "Error opening file" << std::endl;
        return false;
    }

    QTextStream in(&file);

    data = in.readAll().toStdString();

//    std::cout << in.readAll().toStdString() << std::endl;



  //  std::cout << in.readAll().toStdString() << std::endl;

//    std::cout << data << std::endl;


    return true;
}

bool BlobStoreHelper::deleteBlob(const std::string url)
{

    QFile file(url.c_str());
    file.remove();
}

}
