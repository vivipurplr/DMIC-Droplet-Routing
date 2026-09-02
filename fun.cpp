/*
 * fun.cpp
 *
 * Core function file: path planning algorithm implementation.
 * Standalone module for GitHub upload.
 */

#include <iostream>
#include <vector>
#include <algorithm>
#include <QVector>
#include <QMap>
#include <QString>
#include <QFile>
#include <QFileDialog>
#include <QTextStream>
#include <QStringList>

// ============================================================
// Struct Definitions
// ============================================================

// Droplet struct
struct Drop
{
    int id = 0;       // droplet ID
    int row = 0;      // row coordinate
    int col = 0;      // column coordinate
    int width = 0;    // width (grid cells)
    int height = 0;   // height (grid cells)
    int state = 0;    // motion state (0~4)
    int type = 0;     // path type (1~4)

    bool operator==(const Drop d) const
    {
        return row == d.row && col == d.col && height == d.height && width == d.width;
    }
};



struct Path
{
    QVector<Drop> dropInfo;
    int delay = 1000;          // default delay (ms)
    Path(QVector<Drop> drop)
    {
        this->dropInfo = drop;
    }
};

// ============================================================
// Global Variable Declarations
// ============================================================
extern QVector<Path> m_startList;   // start reagent list
extern QVector<Path> m_endList;     // end reagent list
extern QVector<QVector<int>> reg_map;  // 720x1280 occupancy map
extern QVector<Path> stepList;         // path step list

extern int m_up;         // upper avoidance distance
extern int m_down;       // lower avoidance distance
extern int m_upRight;    // upper-half right-channel target row
extern int m_upLeft;     // upper-half left-channel target row
extern int m_downLeft;   // lower-half left-channel target row
extern int m_downRight;  // lower-half right-channel target row

// ============================================================
// Function Implementations
// ============================================================
void UpdateRegMap(Drop &d1, Drop &d2)   // update occupancy map: clear d1, mark d2
{
    for(int i=0;i<d1.height;i++)
    {
        if(d1.row + i <0 || d1.row + i >= 720)
        {
            continue;
        }
        for(int j=0;j<d1.width;j++)
        {
            if(d1.col + j < 0 || d1.col+j >= 1280)
            {
                continue;
            }
            reg_map[d1.row+i][d1.col+j] = 0;
        }
    }

    for(int i=0;i<d2.height;i++)
    {
        if(d2.row + i <0 || d2.row + i >= 720)
        {
            continue;
        }
        for(int j=0;j<d2.width;j++)
        {
            if(d2.col + j < 0 || d2.col+j >= 1280)
            {
                continue;
            }
            reg_map[d2.row+i][d2.col+j] = 1;
        }
    }
}

void LoadChannelConfig()  // load channel position parameters from file
{
    QFile file;
    QString path = QFileDialog::getOpenFileName(nullptr,"Select channel config file","","*.txt");
    if(path.isNull())
    {
        return;
    }
    file.setFileName(path);
    if(!file.open(QFileDevice::ReadOnly))
    {
        return;
    }
    QTextStream in(&file);
    QString str = in.readLine();
    QStringList list = str.split(",");
    m_upLeft = list.at(0).toInt();
    m_upRight = list.at(1).toInt();
    m_downLeft = list.at(2).toInt();
    m_downRight = list.at(3).toInt();
    m_up = list.at(4).toInt();
    m_down = list.at(5).toInt();
    file.close();
}

// ============================================================
// CreatPath: generate movement paths for all droplets
// ============================================================
void CreatPath()
{
    if(m_startList.length()!= m_endList.length())
    {
        return;
    }
    QVector<Drop> starts;
    QMap<int,Drop> start_end;
    int posi=0;
    for(int i=0;i<m_startList.length();i++)
    {
        if(m_startList.at(i).dropInfo.length() != m_endList.at(i).dropInfo.length())
        {
            return;
        }
        for(int j=0;j<m_startList.at(i).dropInfo.length();j++)
        {
            Drop d = m_startList.at(i).dropInfo.at(j);
            d.id = posi;
            starts.append(d);
            start_end.insert(posi,m_endList.at(i).dropInfo.at(j));
            posi++;
        }
    }

    // sort start points
    QVector<Drop> startsup,startsdown;
    for(int i=0;i<starts.length();i++)
    {
        Drop d = starts.at(i);
        d.state = 0;
        if(d.row < 360)
        {
            if(d.col < start_end.value(d.id).col - m_down)
            {
                d.type = 1;
            }
            else
            {
                d.type = 2;
            }
            startsup.append(d);
        }
        else
        {
            if(d.col < start_end.value(d.id).col + start_end.value(d.id).width + m_up)
            {
                d.type = 3;
            }
            else
            {
                d.type = 4;
            }
            startsdown.append(d);
        }
    }
    std::sort(startsup.begin(),startsup.end(),[=](Drop a, Drop b)
    {
        if(a.row == b.row)
        {
            return a.col < b.col;
        }
        else
        {
            return a.row > b.row;
        }
    });
    std::sort(startsdown.begin(),startsdown.end(),[=](Drop a,Drop b)
    {
        if(a.row == b.row)
        {
            return a.col < b.col;
        }
        else
        {
            return a.row < b.row;
        }
    });
    // merge sorted lists
    starts.clear();
    starts.append(startsup);
    starts.append(startsdown);
    // initialize occupancy map
    reg_map.clear();
    reg_map = QVector<QVector<int>>(720,QVector<int>(1280));
    for(int i=0;i<starts.length();i++)
    {
        Drop node = starts.at(i);
        for(int j=0;j<node.height;j++)
        {
            if(node.row + j<0 || node.row + j >= 720)
            {
                continue;
            }
            for(int k=0;k<node.width;k++)
            {
                if(node.col+k<0 || node.col+ k >= 1280)
                {
                    continue;
                }
                reg_map[node.row+j][node.col+k] = 1;
            }
        }
    }
    QVector<Path> path;
    path.append(starts);
    bool flag = true;
    int num = 0;
    while(flag)
    {
        num++;
        flag = false;
        for(int i =0;i<starts.length();i++)
        {
            Drop& s = starts[i];
            Drop s1 = s;
            Drop e = start_end.value(s.id);
            if(s.type == 1)
            {
                if(s.state == 0)
                {
                    if(s.row < m_upRight)
                    {
                        s1.row++;
                        if(moveReg(s,s1)) s = s1;
                    }
                    else
                    {
                        s1.state = 1;
                        s= s1;
                    }
                    flag = true;
                }
                else if(s.state == 1)
                {
                    if(s.col < e.col - m_down)
                    {
                        s1.col++;
                        if(moveReg(s,s1)) s = s1;
                    }
                    else
                    {
                        s1.state = 2;
                        s = s1;
                    }
                    flag = true;
                }
                else if(s.state == 2)
                {
                    if(s.row < e.row)
                    {
                        s1.row++;
                        if(moveReg(s,s1)) s = s1;
                    }
                    else
                    {
                        s1.state = 3;
                        s = s1;
                    }
                    flag = true;
                }
                else if(s.state == 3)
                {
                    if(s.col < e.col)
                    {
                        s1.col++;
                        if(moveReg(s,s1)) s= s1;
                    }
                    else
                    {
                        s1.state = 4;
                        s= s1;
                        s1.row =0;
                        s1.col = 0;
                        s1.width=0;
                        s1.height=0;
                        UpdateRegMap(s,s1);
                    }
                    flag = true;
                }
            }
            else if(s.type == 2)
            {
                if(s.state == 0)
                {
                    if(s.row < m_upLeft)
                    {
                        s1.row++;
                        if(moveReg(s,s1)) s = s1;
                    }
                    else
                    {
                        s1.state = 1;
                        s= s1;
                    }
                    flag = true;
                }
                else if(s.state == 1)
                {
                    if(s.col > e.col - m_down)
                    {
                        s1.col--;
                        if(moveReg(s,s1)) s = s1;
                    }
                    else
                    {
                        s1.state = 2;
                        s = s1;
                    }
                    flag = true;
                }
                else if(s.state == 2)
                {
                    if(s.row < e.row)
                    {
                        s1.row++;
                        if(moveReg(s,s1)) s = s1;
                    }
                    else
                    {
                        s1.state = 3;
                        s = s1;
                    }
                    flag = true;
                }
                else if(s.state == 3)
                {
                    if(s.col < e.col)
                    {
                        s1.col++;
                        if(moveReg(s,s1)) s= s1;
                    }
                    else
                    {
                        s1.state = 4;
                        s= s1;
                        s1.row =0;
                        s1.col = 0;
                        s1.width=0;
                        s1.height=0;
                        UpdateRegMap(s,s1);
                    }
                    flag = true;
                }
            }
            else if(s.type == 3)
            {
                if(s.state == 0)
                {
                    if(s.row > m_downLeft)
                    {
                        s1.row--;
                        if(moveReg(s,s1)) s = s1;
                    }
                    else
                    {
                        s1.state = 1;
                        s= s1;
                    }
                    flag = true;
                }
                else if(s.state == 1)
                {
                    if(s.col < e.col +e.width +  m_up)
                    {
                        s1.col++;
                        if(moveReg(s,s1)) s = s1;
                    }
                    else
                    {
                        s1.state = 2;
                        s = s1;
                    }
                    flag = true;
                }
                else if(s.state == 2)
                {
                    if(s.row > e.row)
                    {
                        s1.row--;
                        if(moveReg(s,s1)) s = s1;
                    }
                    else
                    {
                        s1.state = 3;
                        s = s1;
                    }
                    flag = true;
                }
                else if(s.state == 3)
                {
                    if(s.col > e.col + e.width - s.width)
                    {
                        s1.col--;
                        if(moveReg(s,s1)) s= s1;
                    }
                    else
                    {
                        s1.state = 4;
                        s= s1;
                        s1.row =0;
                        s1.col = 0;
                        s1.width=0;
                        s1.height=0;
                        UpdateRegMap(s,s1);
                    }
                    flag = true;
                }
            }
            else if(s.type == 4)
            {
                if(s.state == 0)
                {
                    if(s.row > m_downRight)
                    {
                        s1.row--;
                        if(moveReg(s,s1)) s = s1;
                    }
                    else
                    {
                        s1.state = 1;
                        s= s1;
                    }
                    flag = true;
                }
                else if(s.state == 1)
                {
                    if(s.col > e.col +e.width +  m_up)
                    {
                        s1.col--;
                        if(moveReg(s,s1)) s = s1;
                    }
                    else
                    {
                        s1.state = 2;
                        s = s1;
                    }
                    flag = true;
                }
                else if(s.state == 2)
                {
                    if(s.row > e.row)
                    {
                        s1.row--;
                        if(moveReg(s,s1)) s = s1;
                    }
                    else
                    {
                        s1.state = 3;
                        s = s1;
                    }
                    flag = true;
                }
                else if(s.state == 3)
                {
                    if(s.col > e.col + e.width - s.width)
                    {
                        s1.col--;
                        if(moveReg(s,s1)) s= s1;
                    }
                    else
                    {
                        s1.state = 4;
                        s= s1;
                        s1.row =0;
                        s1.col = 0;
                        s1.width=0;
                        s1.height=0;
                        UpdateRegMap(s,s1);
                    }
                    flag = true;
                }
            }

        }



        QVector<Drop> list;
        for(int i=0;i<starts.length();i++)
        {
            if(starts.at(i).state == 4)
            {
                continue;
            }
            list.append(starts.at(i));
        }
        path.append(list);
        if(num>5000)
        {
            flag = false;
        }
    }
}


// ============================================================
// moveReg: move droplet with collision detection
// ============================================================
bool moveReg(Drop d1, Drop d2)
{
    bool flag = true;
    for(int i= -2;i<d2.height+2;i++)
    {
        int row = d2.row+i;
        if(row<0 || row>=720) continue;
        for(int j= -2;j<d2.width+2;j++)
        {
            int col = d2.col+j;
            if(col<0 || col>=1280) continue;
            if( row >= d1.row && row < d1.row+d1.height && col >= d1.col && col< d1.col+d1.width) continue;

            if(reg_map[row][col] != 0)
            {
                flag = false;
                break;
            }
        }
        if(!flag)
        {
            break;
        }
    }
    if(!flag)
    {
        return false;
    }

    for(int i=0;i<d1.height;i++)
    {
        if(d1.row + i <0 || d1.row + i >= 720) continue;

        for(int j=0;j<d1.width;j++)
        {
            if(d1.col + j < 0 || d1.col+j >= 1280) continue;

            reg_map[d1.row+i][d1.col+j] = 0;
        }
    }

    for(int i=0;i<d2.height;i++)
    {
        if(d2.row + i <0 || d2.row + i >= 720) continue;

        for(int j=0;j<d2.width;j++)
        {
            if(d2.col + j < 0 || d2.col+j >= 1280) continue;

            reg_map[d2.row+i][d2.col+j] = 1;
        }
    }

    return true;
}

