//���������࣬����Ϊm * n,ֱ�ӵ���
using System;
using UnityEngine;

public class Matrix
{
    double[,] A;
    int m, n;
    string name;
    public Matrix(int am, int an)
    {
        m = am;
        n = an;
        A = new double[m, n];
        name = "Result";
    }
    public Matrix(int am, int an, string aName)
    {
        m = am;
        n = an;
        A = new double[m, n];
        name = aName;
    }

    public int getM
    {
        get { return m; }
    }
    public int getN
    {
        get { return n; }
    }
    public double[,] Detail
    {
        get { return A; }
        set { A = value; }
    }
    public string Name
    {
        get { return name; }
        set { name = value; }
    }

    public void SetRow(int index, Vector4 Row)
    {
        for(int i=0; i< 4; i++)
        {
            A[index, i] = Row[i];
        }
    }

    public void SetColumn(int index, Vector4 Column)
    {
        for (int i = 0; i < 4; i++)
        {
            A[i, index] = Column[i];
        }
    }
}

/***********����ͨ�ò������*************/

class MatrixOperator
{
    //����ӷ�
    public static Matrix MatrixAdd(Matrix Ma, Matrix Mb)
    {
        int m = Ma.getM;
        int n = Ma.getN;
        int m2 = Mb.getM;
        int n2 = Mb.getN;

        if ((m != m2) || (n != n2))
        {
            Exception myException = new Exception("����ά����ƥ��");
            throw myException;
        }

        Matrix Mc = new Matrix(m, n);
        double[,] c = Mc.Detail;
        double[,] a = Ma.Detail;
        double[,] b = Mb.Detail;

        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                c[i, j] = a[i, j] + b[i, j];
        return Mc;
    }

    //�������
    public static Matrix MatrixSub(Matrix Ma, Matrix Mb)
    {
        int m = Ma.getM;
        int n = Ma.getN;
        int m2 = Mb.getM;
        int n2 = Mb.getN;
        if ((m != m2) || (n != n2))
        {
            Exception myException = new Exception("����ά����ƥ��");
            throw myException;
        }
        Matrix Mc = new Matrix(m, n);
        double[,] c = Mc.Detail;
        double[,] a = Ma.Detail;
        double[,] b = Mb.Detail;

        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                c[i, j] = a[i, j] - b[i, j];
        return Mc;
    }

    //����˷�
    public static Matrix MatrixMulti(Matrix Ma, Matrix Mb)
    {
        int m = Ma.getM;
        int n = Ma.getN;
        int m2 = Mb.getM;
        int n2 = Mb.getN;

        if (n != m2)
        {
            Exception myException = new Exception("����ά����ƥ��");
            throw myException;
        }

        Matrix Mc = new Matrix(m, n2);
        double[,] c = Mc.Detail;
        double[,] a = Ma.Detail;
        double[,] b = Mb.Detail;

        for (int i = 0; i < m; i++)
            for (int j = 0; j < n2; j++)
            {
                c[i, j] = 0;
                for (int k = 0; k < n; k++)
                    c[i, j] += a[i, k] * b[k, j];
            }
        return Mc;

    }

    //��������
    public static Matrix MatrixSimpleMulti(double k, Matrix Ma)
    {
        int m = Ma.getM;
        int n = Ma.getN;
        Matrix Mc = new Matrix(m, n);
        double[,] c = Mc.Detail;
        double[,] a = Ma.Detail;

        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                c[i, j] = a[i, j] * k;
        return Mc;
    }

    //����ת��
    public static Matrix MatrixTrans(Matrix MatrixOrigin)
    {
        int m = MatrixOrigin.getM;
        int n = MatrixOrigin.getN;
        Matrix MatrixNew = new Matrix(n, m);
        double[,] c = MatrixNew.Detail;
        double[,] a = MatrixOrigin.Detail;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                c[i, j] = a[j, i];
        return MatrixNew;
    }

    //�������棨������󷨣�
    public static Matrix MatrixInvByCom(Matrix Ma)
    {
        double d = MatrixOperator.MatrixDet(Ma);
        if (d == 0)
        {
            Exception myException = new Exception("û�������");
            throw myException;
        }
        Matrix Ax = MatrixOperator.MatrixCom(Ma);
        Matrix An = MatrixOperator.MatrixSimpleMulti((1.0 / d), Ax);
        return An;
    }
    //��Ӧ����ʽ�Ĵ�������ʽ����
    public static Matrix MatrixSpa(Matrix Ma, int ai, int aj)
    {
        int m = Ma.getM;
        int n = Ma.getN;
        if (m != n)
        {
            Exception myException = new Exception("�����Ƿ���");
            throw myException;
        }
        int n2 = n - 1;
        Matrix Mc = new Matrix(n2, n2);
        double[,] a = Ma.Detail;
        double[,] b = Mc.Detail;

        //����
        for (int i = 0; i < ai; i++)
            for (int j = 0; j < aj; j++)
            {
                b[i, j] = a[i, j];
            }
        //����
        for (int i = ai; i < n2; i++)
            for (int j = aj; j < n2; j++)
            {
                b[i, j] = a[i + 1, j + 1];
            }
        //����
        for (int i = 0; i < ai; i++)
            for (int j = aj; j < n2; j++)
            {
                b[i, j] = a[i, j + 1];
            }
        //����
        for (int i = ai; i < n2; i++)
            for (int j = 0; j < aj; j++)
            {
                b[i, j] = a[i + 1, j];
            }
        //����λ
        if ((ai + aj) % 2 != 0)
        {
            for (int i = 0; i < n2; i++)
                b[i, 0] = -b[i, 0];

        }
        return Mc;

    }

    //���������ʽ,��������Ƿ���
    public static double MatrixDet(Matrix Ma)
    {
        int m = Ma.getM;
        int n = Ma.getN;
        if (m != n)
        {
            Exception myException = new Exception("����ά����ƥ��");
            throw myException;
        }
        double[,] a = Ma.Detail;
        if (n == 1) return a[0, 0];

        double D = 0;
        for (int i = 0; i < n; i++)
        {
            D += a[1, i] * MatrixDet(MatrixSpa(Ma, 1, i));
        }
        return D;
    }

    //����İ������
    public static Matrix MatrixCom(Matrix Ma)
    {
        int m = Ma.getM;
        int n = Ma.getN;
        Matrix Mc = new Matrix(m, n);
        double[,] c = Mc.Detail;
        double[,] a = Ma.Detail;

        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                c[i, j] = MatrixDet(MatrixSpa(Ma, j, i));

        return Mc;
    }
}
