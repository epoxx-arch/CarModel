#include <vector>
#include <random>
#include <iostream>

struct Node{
  std::vector<Node> child{};
  int index = 0;
};

Node FindChild(Node* root){
    return root->child[2];
}


int main(){
    Node *root = new Node();
  root->index = 1 ;
  auto c1 = Node();
    c1.index = 2;
    auto c2 = Node();
    c2.index = 3;
    auto c3 = Node();
    c3.index = 4;
  root->child.push_back(c1);
  root->child.push_back(c2);
  root->child.push_back(c3);
  while (!root->index != 10000){
        std::cout<<root->index<<std::endl;
      auto c = FindChild(root);
      root = &c;
      std::cout<<root->index<<std::endl;
      break;
  }
  return 0;
}