#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>


using std::ifstream;
using std::vector;
using std::string;
using std::getline;
using std::istringstream;
using std::cout;


vector<int> ParseLine(string line){
    auto sline = istringstream(line);
    vector<int> row;
    int n;
    char sep;
    while (sline >> n >> sep && sep == ','){
        row.push_back(n);
    }
    return row;   
}


vector<vector<int>> ReadFromFile(string path){
    vector<vector<int>> board;
    std::ifstream file;
    file.open(path);
    
    string line_str;
    while (getline(file, line_str)){
        vector<int> line = ParseLine(line_str);
        board.push_back(line);
    }
    return board; 
}


void PrintBoard(vector<vector<int>> board){
    for (auto row: board){
        for(auto elt: row){
            cout << elt << " ";
        }
        cout << "\n";
    }
}


int main(){
    vector<vector<int>> board = ReadFromFile("ex.txt");

    PrintBoard(board);



    return EXIT_SUCCESS;
}