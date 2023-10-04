#include "file_saver.h"
#include <deque>
#include <iostream>
#include <fstream>
#include <exception>
#include <algorithm>

namespace{
    // Функция, сохраняющая в csv двумерный массив data с размером строки row_size
    template <typename T>
    void save_csv_to_stream(std::ofstream &output_file, const std::vector<T> &data, const uint32_t row_size)
    {

        for (int row_index = 0; row_index < data.size(); row_index += row_size)
        {
            output_file << std::to_string(data[row_index]);
            for (int column_index = 1; column_index < row_size; column_index++)
            {
                output_file << "," << std::to_string(data[row_index + column_index]);
            }
            output_file << '\n';
        }
    }
}


// Функция, сохраняющая в буффер двумерный массив data с размером строки row_size
template <typename T>
void FileSaver::save_csv(const std::string &out_filename_csv, const std::vector<T> &data, const uint32_t row_size)
{
    std::ofstream ofs(out_filename_csv, std::ofstream::out);
    ::save_csv_to_stream<T>(ofs, data, row_size);
    ofs.close();
}

    


// Функция, Записывающая csv файл в xml под <csv></csv>
template <typename T>
void FileSaver::save_csv_to_xml(const std::string out_filename_xml, const std::string in_filename_xml, const std::vector<T> &data, const uint32_t row_size)
{
    std::ifstream sourceFile(in_filename_xml);
    std::ofstream targetFile(out_filename_xml, std::ofstream::out);
    if (sourceFile.fail())
    {
        throw std::system_error(errno, std::system_category(), "failed to open xml to save csv inside: " + in_filename_xml);
    }
    if (targetFile.fail())
    {
        throw std::system_error(errno, std::system_category(), "failed to open target to save csv inside: " + out_filename_xml);
    }

    int line_count = std::count(std::istreambuf_iterator<char>(sourceFile),
                                std::istreambuf_iterator<char>(), '\n');
    sourceFile.seekg(0, std::ios_base::beg);

    // Копируем содержимое исходного файла в целевой файл
    for (int i = 0; i <= line_count; i++)
    {
        std::string line = "";
        std::getline(sourceFile, line);
        if (line.find("</input_info>") != std::string::npos)
        {
            break;
        }
        targetFile << line << std::endl;

        if (line.find("</scene>") != std::string::npos) // Чтобы не записывать <csv></csv>, если он есть.
        {
            break;
        }
    }

    sourceFile.close();

    targetFile << "<csv>" << std::endl;
    save_csv_to_stream<T>(targetFile, data, row_size);
    targetFile << "</csv>";
    targetFile << "</input_info>";
    targetFile.close();

    std::cout << "XML file created successfully." << std::endl;
}


