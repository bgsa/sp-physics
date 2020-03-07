#include "TestHeader.h"
#include <BinaryTreeNode.h>

#define CLASS_NAME BinaryTreeNodeTest

namespace SP_PHYSICS_TEST_NAMESPACE
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		
		SP_TEST_METHOD_DEF(BinaryTreeNodeTest_childrenCount_Test);

		SP_TEST_METHOD_DEF(BinaryTreeNodeTest_height_Test);

		SP_TEST_METHOD_DEF(BinaryTreeNodeTest_leftHeight_Test);

		SP_TEST_METHOD_DEF(BinaryTreeNodeTest_rightHeight_Test);

		SP_TEST_METHOD_DEF(BinaryTreeNodeTest_level_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeNodeTest_childrenCount_Test)
	{
		BinaryTreeNode<std::string>* node = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		node->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		node->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		node->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		node->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		node->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		node->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));
		
		node->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));

		int childrenCount = node->childrenCount();

		Assert::AreEqual(7, childrenCount, L"wrong value", LINE_INFO());

		ALLOC_RELEASE(node);
	}

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeNodeTest_height_Test)
	{
		BinaryTreeNode<std::string>* node = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		node->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		node->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		node->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		node->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		node->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		node->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));

		node->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));

		int height = node->height();

		Assert::AreEqual(3, height, L"wrong value", LINE_INFO());

		ALLOC_RELEASE(node);
	}

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeNodeTest_leftHeight_Test)
	{
		BinaryTreeNode<std::string>* node = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		node->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		node->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		node->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		node->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		node->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		node->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));

		node->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));

		int height = node->leftHeight();

		Assert::AreEqual(3, height, L"wrong value", LINE_INFO());

		ALLOC_RELEASE(node);
	}

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeNodeTest_rightHeight_Test)
	{
		BinaryTreeNode<std::string>* node = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		node->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		node->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		node->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		node->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		node->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		node->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));

		node->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));

		int height = node->rightHeight();

		Assert::AreEqual(2, height, L"wrong value", LINE_INFO());

		ALLOC_RELEASE(node);
	}

	SP_TEST_METHOD(CLASS_NAME, BinaryTreeNodeTest_level_Test)
	{
		BinaryTreeNode<std::string>* root = ALLOC_NEW(BinaryTreeNode<std::string>)("1");
		root->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("2"));
		root->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("3"));

		root->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("4"));
		root->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("5"));

		root->leftNode()->leftNode()->setLeftNode(ALLOC_NEW(BinaryTreeNode<std::string>)("6"));
		root->leftNode()->leftNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("7"));

		root->rightNode()->setRightNode(ALLOC_NEW(BinaryTreeNode<std::string>)("8"));

		int level = root->level();
		Assert::AreEqual(1, level, L"wrong value", LINE_INFO());

		level = root->rightNode()->level();
		Assert::AreEqual(2, level, L"wrong value", LINE_INFO());

		level = root->rightNode()->rightNode()->level();
		Assert::AreEqual(3, level, L"wrong value", LINE_INFO());

		level = root->leftNode()->leftNode()->leftNode()->level();
		Assert::AreEqual(4, level, L"wrong value", LINE_INFO());

		ALLOC_RELEASE(root);
	}

}

#undef CLASS_NAME