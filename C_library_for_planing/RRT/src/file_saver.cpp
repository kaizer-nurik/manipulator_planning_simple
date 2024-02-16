#include "file_saver.h"
#include <deque>
#include <iostream>
#include <fstream>
#include <exception>
#include <algorithm>
#include <filesystem>
// Функция, сохраняющая дерево из rrt для визуализации в питоне.
void FileSaver::save_rrt_tree(const std::string &output_filename, const RRT::Tree &tree)
{

    std::deque<RRT::Tree::Node*> need_visit;
    std::map<RRT::Tree::Node*, int> node2ind; // словарь ноды в индекс
    need_visit.push_back(tree.head);
    int node_ind = 0;

    node2ind[tree.head] = 0;
    std::ofstream ofs(output_filename, std::ofstream::out | std::ofstream::trunc);
    if (ofs.fail())
    {
        throw std::system_error(errno, std::system_category(), "failed to open csv to save tree: " + output_filename);
    }
    ofs << 0 << "," << tree.head->get_position().configuration[0] << "," << tree.head->get_position().configuration[1] << "," << 0 << std::endl;
    while (need_visit.size() > 0)
    {
        int curr_parent = node2ind[need_visit[0]];
        for (auto child : (*need_visit[0]).children)
        {
            need_visit.push_back(child);
            node_ind++;
            node2ind[child] = node_ind;
            ofs << node_ind << "," << child->get_position().configuration[0] << "," << child->get_position().configuration[1] << "," << curr_parent << std::endl;
        }
        need_visit.pop_front();
    }
    ofs.close();
}

void FileSaver::write_end_config_to_csv(std::string filename, std::vector<Robot> IK_res)
{
    std::ofstream file(filename, std::ofstream::out | std::ofstream::trunc);

    if (file.fail())
    {
        std::cout << "Unable to open the file." << std::endl;
        return;
    }

    for (auto answer : IK_res)
    {
        file << answer.configuration[0];
        for (auto angle = answer.configuration.begin() + 1; angle <= answer.configuration.end(); angle++)
        {
            file << "," << *angle;
        }
        file << std::endl;
    }
}
void FileSaver::write_map_to_json(std::string filename, std::map<std::string, std::string> &stats)
{

    //std::ofstream file(filename, std::ofstream::out | std::ofstream::trunc);
    std::ofstream file(filename, std::ofstream::out | std::ofstream::app);
    if (file.fail())
    {
        std::cout << "Unable to open the file." << std::endl;
        return;
    }

    file << "{" << std::endl;
    int curr_size = 1;
    for (auto [key, value] : stats)
    {
        if (curr_size < stats.size())
        {
            file << "\"" + key + "\":" + "\"" + value + "\"," << std::endl;
        }

        else
        {
            file << "\"" + key + "\":" + "\"" + value  + "\""<< std::endl;
        }
        curr_size++;
    }
    file << "}" << std::endl;
    file.close();
}

