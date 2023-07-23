#pragma once

#include <string>
#include <vector>
#include "planner.h"

// namespace, отвечающий за сохранение файлов
namespace FileSaver
{

    // Функция, сохраняющая в csv двумерный массив data с размером строки row_size
    template <typename T>
    void save_csv(const std::string &out_filename_csv, const std::vector<T> &data, const uint32_t row_size);

    // Функция, Записывающая csv файл в xml под <csv></csv>
    template <typename T>
    void save_csv_to_xml(const std::string out_filename_xml, const std::string in_filename_xml, const std::vector<T> &data, const uint32_t row_size);

    // Функция, сохраняющая дерево из rrt для визуализации в питоне.
    void save_rrt_tree(const std::string &output_filename, const RRT::Tree &tree);

    
}
#include "file_saver.tpp"
