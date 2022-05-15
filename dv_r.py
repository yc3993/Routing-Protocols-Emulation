import sys, time, pickle, threading
from socket import *


def prompt():
    sys.stdout.write(">>> ")
    sys.stdout.flush()

def listen(routingTable, cost):
    graph = cost
    sendingTime = 0
    while True:
        msg, addr = skt.recvfrom(2048)
        temp = pickle.loads(msg)
        print(temp)
        try:
            graph[addr[1]] = temp[2]
            continue
        except:
            newTable = temp[addr[1]]
        flag = 0
        #print(newTable)
        if newTable[addr[1]][0] == 'flag':
            #newTable[addr[1]] = [addr[1], 0]
            flag = 1
            print('test')
            if addr[1] != local_port:
                print('[{}] Link value message received at Node {} from Node {}'.format(time.time(), local_port, addr[1]))
        else: print('[{}] Message received at Node {} from Node {}'.format(time.time(), local_port, addr[1]))
        # print(pickle.loads(msg), addr)
        update = False
        # In every table: change -> only the line which represent the port changed. every other lines changed not count.
        # put new line into routing table

        # whether there are new nodes
        for port in newTable.keys():
            if port not in routingTable[local_port]:
                routingTable[port] = {}
                for node in routingTable.keys():
                    routingTable[node][port] = [None, float('inf')]
                    routingTable[port][node] = [None, float('inf')]
                update = True
                # print(update)

        # routingTable[addr[1]] = newTable
        for port in newTable:
            if routingTable[addr[1]][port] != newTable[port]:
                routingTable[addr[1]][port] = newTable[port]
                update = True

        #print(routingTable)

        # update
        for node in routingTable[local_port].keys():
            if node != local_port:
                routingTable[local_port][node][1] = float('inf')
                for port in graph:
                    if routingTable[local_port][node][1] > graph[port] + routingTable[port][node][1]:
                            routingTable[local_port][node][1] = graph[port] + routingTable[port][node][1]
                            routingTable[local_port][node][0] = port
                            #print(port)
                            #update = True

        # print table
        print('[{}] Node {} Routing Table'.format(time.time(), local_port))
        for port in routingTable[local_port]:
            if port == local_port:
                continue
            if port != routingTable[local_port][port][0]:
                print('- ({}) -> Node {}; Next hop -> Node {}'.format(routingTable[local_port][port][1], port,
                                                                      routingTable[local_port][port][0]))
            else:
                print('- ({}) -> Node {}'.format(routingTable[local_port][port][1], port))

        # If there is any change in the distance vector, a node should send the updated information to its neighbors.
        # print(update,1)
        if update or sendingTime == 0:
            sendingTime += 1
            for port in graph.keys():
                if port != local_port:
                    data = pickle.dumps(routingTable)
                    skt.sendto(data, ('localhost', port))
                    if flag == 1:
                        print('[{}] Link value message sent from Node {} to Node {}'.format(time.time(), local_port, port))
                    else: print('[{}] Message sent from Node {} to Node {}'.format(time.time(), local_port, port))


if __name__ == "__main__":
    #print(sys.argv)
    #mode = sys.argv[2]
    local_port = int(sys.argv[3])
    if local_port <= 1024:
        print('Invalid local-port number')
        exit()

    graph = {}
    lastNode = False
    graph[local_port] = 0
    i = 4
    while i < len(sys.argv):
        if sys.argv[i] == 'last':
            lastNode = True
            i += 1
            costChange = 0
            if i < len(sys.argv):
                costChange = int(sys.argv[i])
            break
        if int(sys.argv[i]) <= 1024:
            print('Invalid local-port number' + sys.argv[i])
            exit()
        graph[int(sys.argv[i])] = int(sys.argv[i+1])
        i+=2
    print(graph)
    routingTable = {}
#Init the routing table
    for port in graph.keys():
        routingTable[port] = {}
        for destination in graph.keys():
            if port == local_port:
                routingTable[port][destination] = [destination, graph[destination]]
            else:
                routingTable[port][destination] = [None, float('inf')]
    #print(routingTable)

#Print the routing table
    print('[{}] Node {} Routing Table'.format(time.time(), local_port))
    for port in routingTable[local_port]:
        if port == local_port:
            continue
        if port != routingTable[local_port][port][0]:
            print('- ({}) -> Node {}; Next hop -> Node {}'.format(routingTable[local_port][port][1], port,
                                                                  routingTable[local_port][port][0]))
        else:
            print('- ({}) -> Node {}'.format(routingTable[local_port][port][1], port))

#Start sending
    skt = socket(AF_INET, SOCK_DGRAM)
    skt.bind(("", local_port))
    if lastNode:
        for port in routingTable.keys():
            if port != local_port:
                data = pickle.dumps(routingTable)
                skt.sendto(data,('localhost', port))
                print('[{}] Message sent from Node <port-{}> to Node <port-{}>'.format(time.time(), local_port, port))

    th = threading.Thread(target=listen, args=(routingTable, graph))
    th.start()
    #listen(routingTable, graph)
    delay = 5
    close_time = time.time() + delay
    while True:
        if time.time() > close_time:
            break
    if lastNode:
        maxi = 1024
        for port in graph:
            if port != local_port:
                maxi = max(port, maxi)
        graph[maxi] = costChange
        msg = [local_port, maxi, costChange]
        gra_change = pickle.dumps(msg)
        skt.sendto(gra_change, ('localhost', maxi))
        #print(routingTable)
        for node in routingTable[local_port].keys():
            if node != local_port:
                routingTable[local_port][node][1] = float('inf')
                for port in graph:
                    if routingTable[local_port][node][1] > graph[port] + routingTable[port][node][1]:
                            routingTable[local_port][node][1] = graph[port] + routingTable[port][node][1]
                            routingTable[local_port][node][0] = port
                            #print(port)
                            update = True
        routingTable[local_port][local_port][0] = 'flag'
        data = pickle.dumps(routingTable)
        for port in graph:
            if port != local_port:
                skt.sendto(data, ('localhost', port))
                print('[{}] Link value message sent from Node {} to Node {}'.format(time.time(), local_port, port))
        print('[{}] Node <port-{}> cost updated to <{}>'.format(time.time(), maxi, costChange))
        print(routingTable)








