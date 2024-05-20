#include "convolutional_code.h"

using namespace std;

// 편의상 layer와 node를 구분합니다. 처음 보면 헷갈릴수도 있습니다..
// node == state 이고, layer == hamming distance == 경로의 가중치(node 와 node 사이)
// 즉 node에는 state(도착지) 정보가 담겨있고, layer에는 weight(가중치) 정보가 담겨있고,
// 이것을 기준으로 다익스트라 알고리즘을 실행하는 것입니다.(각 layer마다)
// 도식화하면 다음과 같습니다
/*
            (node)---layer---(node)---layer---(node)
              00               00               00

            (node)---layer---(node)---layer---(node)
              01               01               01

            (node)---layer---(node)---layer---(node)
              10               10               10

            (node)---layer---(node)---layer---(node)
              11               11               11

    node idx  0                 1               2    ... 500
    layer idx          0                1       ... 499
*/

string received_string;
string decoded_code;
map<string, int> state2idx; // string state 값을 idx로 숫자화
string idx2output[4][2]; // idx -> encoder output
string idx2nextstate[4][2]; // idx -> next_state
int branch_weight[500][4][2]; // (layer, state , zero or one) -> hamming distance
int backtrace_input[501][4][4]; // (node_idx, next_node, now_node) -> (0 or 1) back trace할건데, 최소경로의 input이  0인지 1인지?
int backtrace_node[501][4]; // (node_idx, next_node) -> prev_node state  backtrace에 사용되는, 현재 노드의 이전 노드 찾기
bool visited[501][4]; // (node_idx) -> (T/F) global variable 이므로 0(false)로 초기화.
int branch_metric[501][4]; // 특정 노드까지의 최단 경로값
const int INF=987654321; // 무한대를 나타내는 상수(경로가 없음을 의미)

void __init__(){
    //insert received bits
    freopen("src/received.txt","r",stdin);
    cin >> received_string;
    fclose(stdin);
}

void initializeFsm(){
    //initialize FSM
    state2idx["00"] = 0;
    state2idx["01"] = 1;
    state2idx["10"] = 2;
    state2idx["11"] = 3;
    idx2output[0][0] = "00";
    idx2output[0][1] = "11";
    idx2output[1][0] = "11";
    idx2output[1][1] = "00";
    idx2output[2][0] = "10";
    idx2output[2][1] = "01";
    idx2output[3][0] = "01";
    idx2output[3][1] = "10";
    idx2nextstate[0][0] = "00";
    idx2nextstate[0][1] = "10";
    idx2nextstate[1][0] = "00";
    idx2nextstate[1][1] = "10";
    idx2nextstate[2][0] = "01";
    idx2nextstate[2][1] = "11";
    idx2nextstate[3][0] = "01";
    idx2nextstate[3][1] = "11";
}

void initializeViterbi(){
    //initialize branch_weight -> edge 의 weight임을 유의.
    fill(&branch_weight[0][0][0], &branch_weight[499][3][2], INF);

    //initialize backtrace_input array -> -1: 경로 없음 0 : input 0인 경로 1 : input 1인 경로
    memset(backtrace_input, -1, sizeof(backtrace_input));

    //initialize branch_metric to INF but initial node to 0 (시작점) node의 weight임에 유의.
    fill(&branch_metric[0][0], &branch_metric[500][4], INF);
    branch_metric[0][0] = 0;

    //initialize backtrace_node to -1
    memset(backtrace_node, -1, sizeof(backtrace_node));
}

int hammingDistance(const string& a, const string& b){
    int res = 0;
    for(int i=0;i<2;i++){
        char a_char = a[i];
        char b_char = b[i];
        if(a_char != b_char) res++;
    }
    return res;
}

void viterbiForward(){
    visited[0][0] = true; // 초기상태:
    for(int idx=0;idx<500;idx++){// layer의 idx 0, 1, 2, 3, .... 499
        string now_output = received_string.substr(2*idx,2); // string idx는 2배씩 ex) "01", "10", "11", "00" 이므로 0, 2, 4, 6 ... 으로 접근해야함
        for(int state_idx=0;state_idx<4;state_idx++){
            if(visited[idx][state_idx] == true){
                for(int inp=0;inp<=1;inp++){
                    string next_state =idx2nextstate[state_idx][inp];
                    int next_state_idx = state2idx[next_state];
                    visited[idx+1][next_state_idx] = true;
                    branch_weight[idx][state_idx][inp] = hammingDistance(idx2output[state_idx][inp], now_output);
                    //가중치 계산해서 경로 갱신 like dijkstra
                    if(branch_metric[idx][state_idx] + branch_weight[idx][state_idx][inp] < branch_metric[idx+1][next_state_idx]){
                        branch_metric[idx+1][next_state_idx] = branch_metric[idx][state_idx] + branch_weight[idx][state_idx][inp];
                        backtrace_node[idx+1][next_state_idx] = state_idx; // node 기준 backtrace, 즉 trace 경로를 선택하는 과정
                        backtrace_input[idx+1][next_state_idx][state_idx] = inp; // 그래서 그 경로에서 input이 zero or one? -> 이게 decoded 값임.
                    }
                }
            }
        }
    }
}

void viterbiBackward(){
    stack<int> ans;
    //마지막 node에서의 min metric 및 그에 해당하는 state 찾기
    int min_metric = INF;
    int min_state = -1;
    for(int now_state=0;now_state<4;now_state++){
        if(branch_metric[500][now_state] < min_metric){
            min_metric = branch_metric[500][now_state];
            min_state = now_state;
        }
    }
    //backtrace
    for(int idx=499;idx>=0;idx--){// 이때는 layer_idx를 의미합니다. just backtrace!
        int min_prev_state = backtrace_node[idx+1][min_state];
        ans.push(backtrace_input[idx+1][min_state][min_prev_state]); // 그 경로의 input stack에넣기
        min_state = min_prev_state;
    }
    freopen("bin/decoded_code.txt","w",stdout);
    while(ans.size()){
        int num = ans.top();
        decoded_code += to_string(num);
        ans.pop();
    }
    cout << decoded_code << "\n";
    fclose(stdout);
}

void decode() {
    initializeFsm();
    initializeViterbi();
    viterbiForward();
    viterbiBackward();
}

void encode() {
    string encoded_code = "";
    string state = "00";
    int index = state2idx[state];
    for (char c : decoded_code){
        int now_num = c - '0';
        string output = idx2output[index][now_num];
        string next_state = idx2nextstate[index][now_num];
        encoded_code += output;
        state = next_state;
        index = state2idx[state];
    }
    freopen("bin/encoded_code.txt", "w" ,stdout);
    cout << encoded_code << "\n";
    fclose(stdout);
    freopen("/dev/tty", "w", stdout);
    vector<int> error_idx;
    int err = 0;
    for(int i=0;i<1000;i++){
        if(encoded_code[i] != received_string[i]) {
            error_idx.push_back(i);
            err++;
        }
    }
    cout << "error : " << err << "\n";
    cout << "error_idx : ";
    for(int idx : error_idx){
        cout << idx << " ";
    }
    cout << "\n";
    return;
}