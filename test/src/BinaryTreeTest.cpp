#include "SpectrumPhysicsTest.h"
#include <BinaryTree.h>

#define CLASS_NAME BinaryTreeTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(BinaryTreeTest_nodeCount_Test);

		SP_TEST_METHOD_DEF(BinaryTreeTest_height_Test);

		SP_TEST_METHOD_DEF(BinaryTreeTest_levels_Test);
		
		SP_TEST_METHOD_DEF(BinaryTreeTest_listPreOrder_Test);

		SP_TEST_METHOD_DEF(BinaryTreeTest_listPostOrder_Test);

		SP_TEST_METHOD_DEF(BinaryTreeTest_listInOrder_Test);

		SP_TEST_METHOD_DEF(BinaryTreeTest_listLevelOrder_Test);

		SP_TEST_METHOD_DEF(BinaryTreeTest_leafs_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeTest_nodeCount_Test)
	{
		BinaryTreeNode<std::string>* root = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		root->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		root->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		root->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		root->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		root->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		root->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));

		root->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));

		BinaryTree<std::string>* tree = ALLOC_NEW(BinaryTree<std::string>)(root);

		int nodeCount = tree->nodeCount();

		Assert::AreEqual(8, nodeCount, L"wrong value", LINE_INFO());

		ALLOC_RELEASE(root);
	}

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeTest_height_Test)
	{
		BinaryTreeNode<std::string>* root = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		root->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		root->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		root->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		root->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		root->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		root->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));

		root->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));

		BinaryTree<std::string>* tree = ALLOC_NEW(BinaryTree<std::string>)(root);

		int height = tree->height();

		Assert::AreEqual(3, height, L"wrong value", LINE_INFO());

		ALLOC_RELEASE(root);
	}

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeTest_levels_Test)
	{
		BinaryTreeNode<std::string>* root = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		root->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		root->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		root->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		root->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		root->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		root->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));

		root->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));
		
		BinaryTree<std::string>* tree = ALLOC_NEW(BinaryTree<std::string>)(root);
		int levels = tree->levels();
		Assert::AreEqual(4, levels, L"wrong value", LINE_INFO());
		ALLOC_RELEASE(tree);

		tree = ALLOC_NEW(BinaryTree<std::string>)(nullptr);
		levels = tree->levels();
		Assert::AreEqual(0, levels, L"wrong value", LINE_INFO());
		ALLOC_RELEASE(tree);

		ALLOC_RELEASE(root);
	}

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeTest_listPreOrder_Test)
	{
		BinaryTreeNode<std::string>* root = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		root->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		root->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		root->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		root->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		root->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		root->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));

		root->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));

		BinaryTree<std::string>* tree = ALLOC_NEW(BinaryTree<std::string>)(root);
		std::vector<std::string> list = tree->listPreOrder();

		std::string expected[8] = {
			"1", "2", "4", "6", "7", "5" , "3" , "8"
		};

		for (size_t i = 0; i < 8; i++)
			Assert::AreEqual(list[i], expected[i], L"wrong value", LINE_INFO());

		ALLOC_RELEASE(root);
	}

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeTest_listPostOrder_Test)
	{
		BinaryTreeNode<std::string>* root = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		root->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		root->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		root->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		root->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		root->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		root->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));

		root->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));

		BinaryTree<std::string>* tree = ALLOC_NEW(BinaryTree<std::string>)(root);
		std::vector<std::string> list = tree->listPostOrder();

		std::string expected[8] = {
			"6", "7", "4", "5", "2", "8" , "3" , "1"
		};

		for (size_t i = 0; i < 8; i++)
			Assert::AreEqual(list[i], expected[i], L"wrong value", LINE_INFO());

		ALLOC_RELEASE(root);
	}

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeTest_listInOrder_Test)
	{
		BinaryTreeNode<std::string>* root = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		root->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		root->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		root->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		root->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		root->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		root->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));

		root->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));

		BinaryTree<std::string>* tree = ALLOC_NEW(BinaryTree<std::string>)(root);
		std::vector<std::string> list = tree->listInOrder();

		std::string expected[8] = {
			"6", "4", "7", "2", "5", "1" , "3" , "8"
		};

		for (size_t i = 0; i < 8; i++)
			Assert::AreEqual(list[i], expected[i], L"wrong value", LINE_INFO());

		ALLOC_RELEASE(root);
	}

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeTest_listLevelOrder_Test)
	{
		BinaryTreeNode<std::string>* root = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		root->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		root->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		root->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		root->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		root->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		root->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));

		root->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));

		BinaryTree<std::string>* tree = ALLOC_NEW(BinaryTree<std::string>)(root);
		std::vector<std::string> list = tree->listLevelOrder();

		std::string expected[8] = {
			"1", "2", "3", "4", "5", "8" , "6" , "7"
		};

		for (size_t i = 0; i < 8; i++)
			Assert::AreEqual(list[i], expected[i], L"wrong value", LINE_INFO());

		ALLOC_RELEASE(root);
	}

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeTest_leafs_Test)
	{
		BinaryTreeNode<std::string>* root = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		root->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		root->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		root->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		root->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		root->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		root->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));

		root->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));

		BinaryTree<std::string>* tree = ALLOC_NEW(BinaryTree<std::string>)(root);
		std::vector<std::string> list = tree->leafs();

		std::string expected[4] = {
			"6", "7", "5", "8"
		};

		Assert::AreEqual(size_t(4), list.size(), L"wrong value", LINE_INFO());

		for (size_t i = 0; i < 4; i++)
			Assert::AreEqual(list[i], expected[i], L"wrong value", LINE_INFO());

		ALLOC_RELEASE(root);
	}

}

#undef CLASS_NAME