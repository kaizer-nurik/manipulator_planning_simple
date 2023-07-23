#include "file_saver.h"
#include <deque>
#include <iostream>
#include <fstream>
#include <exception>
#include <algorithm>


// Функция, сохраняющая дерево из rrt для визуализации в питоне.
void FileSaver::save_rrt_tree(const std::string &output_filename, const RRT::Tree &tree)
{

    std::deque<std::shared_ptr<RRT::Tree::Node>> need_visit;
    std::map<std::shared_ptr<RRT::Tree::Node>, int> node2ind; // словарь ноды в индекс
    need_visit.push_back(tree.head);
    int node_ind = 0;

    node2ind[tree.head] = 0;
    std::ofstream ofs(output_filename, std::ofstream::out);
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
